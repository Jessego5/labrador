// This is the implementation of the five skill nodes. Each one builds its goal from the node's
// ports and the executor's world model, and after the skill finishes it updates that model: what
// was detected, what the gripper holds, which objects have been put away. The executor only
// learns about the scene through these results, so a wrong update here shows up later as a pick
// of something that is not there.
#include "cell_executor/skill_nodes.hpp"

#include <cmath>

namespace cell_executor
{

using cell_interfaces::msg::SkillError;

std::string errorName(uint8_t code)
{
  switch (code) {
    case SkillError::SUCCESS: return "SUCCESS";
    case SkillError::PLAN_FAILED: return "PLAN_FAILED";
    case SkillError::EXECUTION_FAILED: return "EXECUTION_FAILED";
    case SkillError::GRASP_FAILED: return "GRASP_FAILED";
    case SkillError::RELEASE_FAILED: return "RELEASE_FAILED";
    case SkillError::INSERT_JAMMED: return "INSERT_JAMMED";
    case SkillError::TIMEOUT: return "TIMEOUT";
    case SkillError::CANCELED: return "CANCELED";
    case SkillError::INVALID_GOAL: return "INVALID_GOAL";
    case SkillError::UNKNOWN_TARGET: return "UNKNOWN_TARGET";
    case SkillError::NOT_HOLDING: return "NOT_HOLDING";
    case SkillError::NOT_READY: return "NOT_READY";
    case SkillError::PERCEPTION_FAILED: return "PERCEPTION_FAILED";
  }
  return "ERROR_" + std::to_string(code);
}

namespace
{

double round3(double v)
{
  return std::round(v * 1000.0) / 1000.0;
}

}  // namespace

// DetectObjects

DetectObjectsNode::DetectObjectsNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx,
  std::shared_ptr<SkillClients> skills)
: SkillNode(name, config, ctx, skills, skills->detect, "DetectObjects")
{
}

BT::PortsList DetectObjectsNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("class", "", "only objects of this class (empty = all)"),
    BT::InputPort<std::string>("color", "", "only objects of this colour (empty = all)"),
  };
}

std::optional<std::string> DetectObjectsNode::makeGoal(Goal & goal)
{
  goal.object_class = getInput<std::string>("class").value_or("");
  goal.color = getInput<std::string>("color").value_or("");
  filter_.clear();
  if (!goal.object_class.empty()) {
    filter_["class"] = goal.object_class;
  }
  if (!goal.color.empty()) {
    filter_["color"] = goal.color;
  }
  target_ = encodeFilter(filter_);
  return std::nullopt;
}

void DetectObjectsNode::onResult(bool success, const Result * result, Json & record)
{
  if (!success || !result) {
    return;
  }
  ctx_->world.updateDetections(result->objects, filter_);
  Json ids = Json::array();
  for (const auto & o : result->objects) {
    ids.push_back(o.id);
  }
  record["objects"] = ids;
  ctx_->info("detected " + std::to_string(result->objects.size()) + " object(s)");
}

// Pick

PickNode::PickNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx,
  std::shared_ptr<SkillClients> skills)
: SkillNode(name, config, ctx, skills, skills->pick, "Pick")
{
}

BT::PortsList PickNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("object", "detection id or alias (peg_0), may use $item"),
    BT::InputPort<std::string>("grasp", "", "grasp strategy (empty = by object class)"),
  };
}

std::optional<std::string> PickNode::makeGoal(Goal & goal)
{
  const std::string ref = expand(getInput<std::string>("object").value_or(""));
  target_ = ref;
  object_.clear();
  if (ctx_->world.held) {
    return "already holding " + *ctx_->world.held;
  }
  const auto object = ctx_->world.find(ref);
  if (!object) {
    return "no detected object '" + ref + "' (run detect first?)";
  }
  if (ctx_->world.handled(object->id)) {
    return "object " + object->id + " has already been put away";
  }
  object_ = object->id;
  target_ = object->id;
  goal.object_id = object->id;
  goal.object_pose = object->pose;
  goal.object_size = object->size;
  goal.grasp_strategy = getInput<std::string>("grasp").value_or("");
  if (goal.grasp_strategy.empty()) {
    const auto it = ctx_->grasp_strategies.find(object->object_class);
    if (it != ctx_->grasp_strategies.end()) {
      goal.grasp_strategy = it->second;
    }
  }
  return std::nullopt;
}

void PickNode::onResult(bool success, const Result * result, Json & record)
{
  if (object_.empty()) {
    return;
  }
  record["object"] = object_;
  if (success) {
    ctx_->world.held = object_;
  } else if (ctx_->world.held == object_) {
    ctx_->world.held.reset();
  }
  if (result) {
    record["gripper_position"] = round3(result->gripper_position);
  }
}

// Place

PlaceNode::PlaceNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx,
  std::shared_ptr<SkillClients> skills)
: SkillNode(name, config, ctx, skills, skills->place, "Place")
{
}

BT::PortsList PlaceNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("target", "workspace target, may use $index / $item"),
    BT::InputPort<std::string>("compliance", "default", "true, false or default"),
  };
}

std::optional<std::string> PlaceNode::makeGoal(Goal & goal)
{
  const std::string wanted = expand(getInput<std::string>("target").value_or(""));
  target_ = wanted;
  if (!ctx_->world.held) {
    return "not holding anything";
  }
  if (ctx_->workspace) {
    ctx_->workspace->placePose(wanted);  // throws for unknown targets
  }
  target_ = ctx_->allocateTarget(wanted);
  goal.target_name = target_;
  // By default precise targets such as tray slots get a compliant placement and bins get
  // position control. A compliant release on a bin floor jammed: the cube stayed pinned under
  // the adaptive 2F-85 pads, which dragged on it while opening, so the gripper settled at about
  // 0.36 rad instead of the free 0.31 and lifting met about 20 N.
  const std::string compliance = getInput<std::string>("compliance").value_or("default");
  const bool is_bin =
    ctx_->workspace && ctx_->workspace->findBin(wanted.substr(0, wanted.find('/')));
  goal.use_compliance = compliance == "default" ? ctx_->place_compliance && !is_bin :
    compliance == "true";
  return std::nullopt;
}

void PlaceNode::onResult(bool success, const Result * result, Json & record)
{
  (void)result;
  if (!ctx_->world.held) {
    return;
  }
  record["object"] = *ctx_->world.held;
  if (success) {
    ctx_->world.markHandled(*ctx_->world.held);
    ctx_->world.held.reset();
  }
}

// Insert

InsertNode::InsertNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx,
  std::shared_ptr<SkillClients> skills)
: SkillNode(name, config, ctx, skills, skills->insert, "Insert")
{
}

BT::PortsList InsertNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("target", "fixture name"),
    BT::InputPort<double>("max_force", 0.0, "N, push force that counts as jammed (0 = default)"),
    BT::InputPort<int>("max_attempts", 0, "attempts (0 = default)"),
  };
}

std::optional<std::string> InsertNode::makeGoal(Goal & goal)
{
  target_ = expand(getInput<std::string>("target").value_or(""));
  if (!ctx_->world.held) {
    return "not holding anything";
  }
  goal.target_name = target_;
  goal.max_force = static_cast<float>(getInput<double>("max_force").value_or(0.0));
  goal.max_attempts = static_cast<uint8_t>(getInput<int>("max_attempts").value_or(0));
  return std::nullopt;
}

void InsertNode::onResult(bool success, const Result * result, Json & record)
{
  if (ctx_->world.held) {
    record["object"] = *ctx_->world.held;
  }
  if (result) {
    record["depth_reached_mm"] = round3(1000.0 * result->depth_reached);
    record["attempts"] = result->attempts;
    record["max_force_n"] = round3(result->max_force_seen);
  }
  if (success && ctx_->world.held) {
    ctx_->world.markHandled(*ctx_->world.held);
    ctx_->world.held.reset();
  }
}

// MoveToNamedPose

MoveToNamedPoseNode::MoveToNamedPoseNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx,
  std::shared_ptr<SkillClients> skills)
: SkillNode(name, config, ctx, skills, skills->move, "MoveToNamedPose")
{
}

BT::PortsList MoveToNamedPoseNode::providedPorts()
{
  return {BT::InputPort<std::string>("pose", "home", "SRDF named pose")};
}

std::optional<std::string> MoveToNamedPoseNode::makeGoal(Goal & goal)
{
  goal.name = expand(getInput<std::string>("pose").value_or("home"));
  target_ = goal.name;
  return std::nullopt;
}

void registerSkillNodes(
  BT::BehaviorTreeFactory & factory, std::shared_ptr<RunContext> ctx,
  std::shared_ptr<SkillClients> skills)
{
  factory.registerNodeType<DetectObjectsNode>("DetectObjects", ctx, skills);
  factory.registerNodeType<PickNode>("Pick", ctx, skills);
  factory.registerNodeType<PlaceNode>("Place", ctx, skills);
  factory.registerNodeType<InsertNode>("Insert", ctx, skills);
  factory.registerNodeType<MoveToNamedPoseNode>("MoveToNamedPose", ctx, skills);
}

}  // namespace cell_executor
