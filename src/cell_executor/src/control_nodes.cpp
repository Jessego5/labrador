// This is the implementation of the task-structure nodes, Step, ForEachObject and IsHolding.
// Step and ForEachObject write a log record for each finished step and each skipped object, and
// keep /executor/state's step and item up to date through the RunContext.
#include "cell_executor/control_nodes.hpp"

#include <cmath>

namespace cell_executor
{

namespace
{

double rounded(double seconds)
{
  return std::round(seconds * 100.0) / 100.0;
}

}  // namespace

// Step

StepNode::StepNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx)
: BT::DecoratorNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList StepNode::providedPorts()
{
  return {
    BT::InputPort<int>("index", "1-based step number"),
    BT::InputPort<std::string>("title", "", "what the step does"),
    BT::InputPort<std::string>("on_failure", "safe_stop", "safe_stop, skip_step or abort"),
  };
}

BT::NodeStatus StepNode::tick()
{
  const int index = getInput<int>("index").value_or(0);
  const std::string title = getInput<std::string>("title").value_or("");
  if (status() == BT::NodeStatus::IDLE) {
    started_ = ctx_->elapsed();
    ctx_->last_error.clear();
    ctx_->updateProgress([&](Progress & p) {
        p.step = index;
        p.step_name = title;
        p.item.clear();
        p.skill.clear();
        p.phase.clear();
      });
    ctx_->info("step " + std::to_string(index) + ": " + title);
  }
  setStatus(BT::NodeStatus::RUNNING);

  const BT::NodeStatus child = child_node_->executeTick();
  if (child == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  resetChild();
  const double duration = rounded(ctx_->elapsed() - started_);
  if (child == BT::NodeStatus::SUCCESS || child == BT::NodeStatus::SKIPPED) {
    ++ctx_->steps_succeeded;
    ctx_->log({{"event", "step"}, {"step", index}, {"title", title}, {"result", "SUCCESS"},
        {"duration_s", duration}});
    return BT::NodeStatus::SUCCESS;
  }

  FailurePolicy policy = FailurePolicy::SafeStop;
  try {
    policy = parseFailurePolicy(getInput<std::string>("on_failure").value_or("safe_stop"));
  } catch (const std::invalid_argument & e) {
    ctx_->error(e.what());
  }
  ctx_->log({{"event", "step"}, {"step", index}, {"title", title}, {"result", "FAILURE"},
      {"error", ctx_->last_error}, {"policy", toString(policy)}, {"duration_s", duration}});
  const std::string why = ctx_->last_error.empty() ? "" : " (" + ctx_->last_error + ")";
  if (policy == FailurePolicy::SkipStep) {
    ++ctx_->steps_skipped;
    ctx_->warn("step " + std::to_string(index) + " failed" + why + "; skipping it");
    return BT::NodeStatus::SUCCESS;
  }
  ctx_->failed_step = index;
  ctx_->safe_stopped = policy == FailurePolicy::SafeStop;
  ctx_->error("step " + std::to_string(index) + " failed" + why +
    (ctx_->safe_stopped ? "; safe stop" : "; aborting"));
  return BT::NodeStatus::FAILURE;
}

// ForEachObject

ForEachObjectNode::ForEachObjectNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx)
: BT::DecoratorNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList ForEachObjectNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("filter", "", "field=value;... with fields class, color, type"),
    BT::InputPort<int>("max", 0, "at most this many objects (0 = all)"),
    BT::InputPort<std::string>("on_failure", "safe_stop", "skip_step skips a failed object"),
  };
}

BT::NodeStatus ForEachObjectNode::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    ObjectFilter filter;
    try {
      filter = decodeFilter(getInput<std::string>("filter").value_or(""));
    } catch (const std::invalid_argument & e) {
      ctx_->last_error = std::string("for_each: ") + e.what();
      return BT::NodeStatus::FAILURE;
    }
    items_ = ctx_->world.select(filter, getInput<int>("max").value_or(0));
    current_ = 0;
    item_started_ = false;
    if (items_.empty()) {
      ctx_->warn("for_each: no detected objects match '" + encodeFilter(filter) + "'");
      return BT::NodeStatus::SUCCESS;
    }
    std::string list;
    for (const auto & id : items_) {
      list += (list.empty() ? "" : ", ") + id;
    }
    ctx_->info("for_each: " + std::to_string(items_.size()) + " object(s): " + list);
  }
  setStatus(BT::NodeStatus::RUNNING);

  if (!item_started_) {
    const std::string & id = items_[current_];
    config().blackboard->set("item", id);
    config().blackboard->set("index", std::to_string(current_ + 1));
    ctx_->last_error.clear();
    ctx_->updateProgress([&](Progress & p) {p.item = id;});
    item_started_ = true;
  }

  const BT::NodeStatus child = child_node_->executeTick();
  if (child == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  resetChild();
  item_started_ = false;
  if (child == BT::NodeStatus::FAILURE) {
    const bool skip = getInput<std::string>("on_failure").value_or("") == "skip_step";
    if (!skip) {
      return BT::NodeStatus::FAILURE;
    }
    ++ctx_->items_skipped;
    ctx_->warn("for_each: skipping " + items_[current_] +
      (ctx_->last_error.empty() ? "" : " (" + ctx_->last_error + ")"));
    ctx_->log({{"event", "item_skipped"}, {"item", items_[current_]},
        {"error", ctx_->last_error}});
  }
  if (++current_ >= items_.size()) {
    ctx_->updateProgress([](Progress & p) {p.item.clear();});
    return BT::NodeStatus::SUCCESS;
  }
  // go back to the tick loop between objects, so a pause or abort takes effect at a clean point
  return BT::NodeStatus::RUNNING;
}

void ForEachObjectNode::halt()
{
  items_.clear();
  current_ = 0;
  item_started_ = false;
  BT::DecoratorNode::halt();
}

// IsHolding

IsHoldingNode::IsHoldingNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx)
: BT::ConditionNode(name, config), ctx_(std::move(ctx))
{
}

BT::NodeStatus IsHoldingNode::tick()
{
  return ctx_->world.held ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void registerControlNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<RunContext> ctx)
{
  factory.registerNodeType<StepNode>("Step", ctx);
  factory.registerNodeType<ForEachObjectNode>("ForEachObject", ctx);
  factory.registerNodeType<IsHoldingNode>("IsHolding", ctx);
}

}  // namespace cell_executor
