// This is the implementation of the skill server declared in skill_server.hpp: the setup that
// waits for move_group and the controllers, the three skills, and the motion and gripper
// primitives the skills are built from. Every skill runs in a thread of its own and goes through
// one wrapper, run(), so they all succeed, fail and report in the same way, with a SkillError
// code and a message. Some of the rules in here were learnt in simulation, the release that opens
// only part way and the retreat before a failure is reported, and the comments next to them say
// what went wrong.
#include "cell_skills/skill_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cell_skills
{

using namespace std::chrono_literals;
using moveit::core::MoveItErrorCode;

namespace
{

// one number formatted printf-style, for the messages in a SkillFailure
std::string fmt(const char * format, double value)
{
  char buf[128];
  std::snprintf(buf, sizeof(buf), format, value);
  return buf;
}

}  // namespace

// Setup

SkillServer::SkillServer(const rclcpp::NodeOptions & options)
: Node("skills", rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true))
{
  arm_group_ = param<std::string>("arm_group", "arm");
  tcp_link_ = param<std::string>("tcp_link", "tcp");
  gripper_joint_ = param<std::string>("gripper_joint", "robotiq_85_left_knuckle_joint");
  velocity_scaling_ = param("velocity_scaling", 0.3);
  acceleration_scaling_ = param("acceleration_scaling", 0.3);
  cartesian_velocity_scaling_ = param("cartesian_velocity_scaling", 0.1);
  approach_height_ = param("approach_height", 0.10);
  release_clearance_ = param("release_clearance", 0.003);
  gripper_open_ = param("gripper_open", 0.0);
  gripper_close_ = param("gripper_close", 0.8);
  grasp_closed_threshold_ = param("grasp_closed_threshold", 0.75);
  // Release by opening only this far past the grip, and open fully after retreating. An open
  // 2F-85 is about 157 mm across the fingertips, and inside a bin its outer links hit the walls.
  release_opening_ = param("release_opening", 0.25);
  release_min_opening_ = param("release_min_opening", 0.1);
  gripper_timeout_ = param("gripper_timeout", 5.0);
  grasp_params_.grasp_depth = param("grasp_depth", grasp_params_.grasp_depth);
  grasp_params_.fingertip_reach = param("fingertip_reach", grasp_params_.fingertip_reach);
  grasp_params_.surface_clearance = param("surface_clearance", grasp_params_.surface_clearance);

  const auto workspace_file = param<std::string>(
    "workspace_file",
    ament_index_cpp::get_package_share_directory("cell_description") + "/config/workspace.yaml");
  workspace_ = cell_workspace::loadWorkspaceFile(workspace_file);

  // The clients are spread over separate mutually exclusive callback groups. An action client in
  // a reentrant group can handle its goal response and its result on two threads at once, and
  // then it drops the result.
  server_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  gripper_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  state_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = state_group_;
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](const sensor_msgs::msg::JointState & msg) {
      for (std::size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i) {
        if (msg.name[i] == gripper_joint_) {
          std::lock_guard<std::mutex> lock(gripper_mutex_);
          gripper_position_ = msg.position[i];
        }
      }
    },
    sub_options);
  gripper_client_ = rclcpp_action::create_client<GripperCommand>(
    this, param<std::string>("gripper_action", "/gripper_controller/gripper_cmd"), gripper_group_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  move_to_named_pose_server_ =
    makeServer<MoveToNamedPose>("~/move_to_named_pose", &SkillServer::executeMoveToNamedPose);
  pick_server_ = makeServer<Pick>("~/pick", &SkillServer::executePick);
  place_server_ = makeServer<Place>("~/place", &SkillServer::executePlace);

  // MoveGroupInterface needs shared_from_this() and waits for move_group, so it is built after
  // construction, on a thread of its own and off the executor.
  init_timer_ = create_wall_timer(
    100ms, [this]() {
      init_timer_->cancel();
      std::thread([this]() {initialize();}).detach();
    });
}

template<typename T>
T SkillServer::param(const std::string & name, const T & default_value)
{
  // the parameter may already exist, declared automatically from a launch override
  if (!has_parameter(name)) {
    declare_parameter<T>(name, default_value);
  }
  return get_parameter(name).get_value<T>();
}

void SkillServer::initialize()
{
  try {
    // Do not report ready until both controllers can take commands. At startup they are spawned
    // after move_group, and MoveIt fails executions sent before they exist. Until then every
    // goal is answered with NOT_READY.
    const auto timeout = std::chrono::duration<double>(param("controller_wait_timeout", 120.0));
    auto arm_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      this, param<std::string>(
        "arm_controller_action", "/joint_trajectory_controller/follow_joint_trajectory"),
      gripper_group_);
    if (!arm_client->wait_for_action_server(timeout) ||
      !gripper_client_->wait_for_action_server(timeout))
    {
      throw std::runtime_error("arm or gripper controller action server not available");
    }

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), arm_group_);
    move_group_->setEndEffectorLink(tcp_link_);
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(3);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    planning_frame_ = move_group_->getPlanningFrame();
    addCollisionObjects();
    ready_ = true;
    RCLCPP_INFO(
      get_logger(), "Skill server ready (group '%s', tcp '%s', planning frame '%s')",
      arm_group_.c_str(), tcp_link_.c_str(), planning_frame_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Skill server failed to initialise: %s", e.what());
  }
}

void SkillServer::addCollisionObjects()
{
  using moveit_msgs::msg::CollisionObject;
  using shape_msgs::msg::SolidPrimitive;
  std::vector<CollisionObject> objects;
  const auto box = [&](const std::string & id, const Eigen::Vector3d & center,
      const Eigen::Vector3d & size) {
      CollisionObject o;
      o.header.frame_id = workspace_.frame;
      o.id = id;
      SolidPrimitive p;
      p.type = SolidPrimitive::BOX;
      p.dimensions = {size.x(), size.y(), size.z()};
      geometry_msgs::msg::Pose pose;
      pose.position.x = center.x();
      pose.position.y = center.y();
      pose.position.z = center.z();
      pose.orientation.w = 1.0;
      o.primitives.push_back(p);
      o.primitive_poses.push_back(pose);
      o.operation = CollisionObject::ADD;
      objects.push_back(o);
    };

  // the table top, matching mjcf/scene.xml, 2 mm below z = 0 so the robot base is not touching it
  const auto table_size = param<std::vector<double>>("table_size", {1.10, 1.30, 0.04});
  const auto table_center = param<std::vector<double>>("table_center", {0.35, 0.0, -0.022});
  box("table", {table_center[0], table_center[1], table_center[2]},
    {table_size[0], table_size[1], table_size[2]});

  // Trays and fixtures go in as solid boxes over their footprints. The final descents into
  // pockets and bins are short vertical moves planned without collision checking, so the solid
  // boxes do not get in their way.
  for (const auto & t : workspace_.trays) {
    const auto fp = t.footprint();
    box(t.name, {0.5 * (fp.min.x() + fp.max.x()), 0.5 * (fp.min.y() + fp.max.y()), 0.5 * t.origin.z()},
      {fp.max.x() - fp.min.x(), fp.max.y() - fp.min.y(), t.origin.z()});
  }
  // bins are open boxes, a floor and four walls as in the MJCF, so the fingers may go inside
  constexpr double wall = 0.005;
  for (const auto & b : workspace_.bins) {
    const Eigen::Vector3d c = b.center;
    const Eigen::Vector3d s = b.size;
    box(b.name + "_floor", {c.x(), c.y(), 0.5 * c.z()}, {s.x(), s.y(), c.z()});
    box(b.name + "_wall_xn", {c.x() - 0.5 * (s.x() - wall), c.y(), 0.5 * s.z()}, {wall, s.y(), s.z()});
    box(b.name + "_wall_xp", {c.x() + 0.5 * (s.x() - wall), c.y(), 0.5 * s.z()}, {wall, s.y(), s.z()});
    box(b.name + "_wall_yn", {c.x(), c.y() - 0.5 * (s.y() - wall), 0.5 * s.z()}, {s.x(), wall, s.z()});
    box(b.name + "_wall_yp", {c.x(), c.y() + 0.5 * (s.y() - wall), 0.5 * s.z()}, {s.x(), wall, s.z()});
  }
  for (const auto & f : workspace_.fixtures) {
    const auto fp = f.footprint();
    const double h = f.hole_pose.translation().z();
    box(f.name, {0.5 * (fp.min.x() + fp.max.x()), 0.5 * (fp.min.y() + fp.max.y()), 0.5 * h},
      {fp.max.x() - fp.min.x(), fp.max.y() - fp.min.y(), h});
  }

  moveit::planning_interface::PlanningSceneInterface scene;
  if (!scene.applyCollisionObjects(objects)) {
    throw std::runtime_error("failed to add collision objects to the planning scene");
  }
  RCLCPP_INFO(get_logger(), "Added %zu collision objects to the planning scene", objects.size());
}

template<typename ActionT>
typename rclcpp_action::Server<ActionT>::SharedPtr SkillServer::makeServer(
  const std::string & name,
  void (SkillServer::* execute)(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>))
{
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  return rclcpp_action::create_server<ActionT>(
    this, name,
    [this, name](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>) {
      bool expected = false;
      if (!busy_.compare_exchange_strong(expected, true)) {
        RCLCPP_WARN(get_logger(), "Rejecting %s goal: another skill is running", name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
    [this, execute](const std::shared_ptr<GoalHandle> goal_handle) {
      // a skill blocks on MoveIt and the gripper, so each one runs in its own thread
      std::thread([this, execute, goal_handle]() {(this->*execute)(goal_handle);}).detach();
    },
    rcl_action_server_get_default_options(), server_group_);
}

template<typename ActionT, typename Body>
void SkillServer::run(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
  const std::string & skill, Body && body)
{
  auto result = std::make_shared<typename ActionT::Result>();
  const auto start = std::chrono::steady_clock::now();
  try {
    if (!ready_) {
      throw SkillFailure(SkillError::NOT_READY, "skill server is still initialising");
    }
    body(*result);
    result->success = true;
    result->error.code = SkillError::SUCCESS;
  } catch (const SkillFailure & f) {
    result->success = false;
    result->error.code = f.code;
    result->message = f.what();
  } catch (const std::exception & e) {
    result->success = false;
    result->error.code = SkillError::EXECUTION_FAILED;
    result->message = e.what();
  }
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

  if (result->success) {
    RCLCPP_INFO(get_logger(), "%s succeeded in %.2f s", skill.c_str(), secs);
    goal_handle->succeed(result);
  } else if (result->error.code == SkillError::CANCELED && goal_handle->is_canceling()) {
    RCLCPP_WARN(get_logger(), "%s canceled after %.2f s", skill.c_str(), secs);
    goal_handle->canceled(result);
  } else {
    RCLCPP_WARN(
      get_logger(), "%s failed after %.2f s (code %u): %s", skill.c_str(), secs,
      result->error.code, result->message.c_str());
    goal_handle->abort(result);
  }
  busy_ = false;
}

template<typename ActionT>
void SkillServer::feedback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
  const std::string & phase, float progress)
{
  auto fb = std::make_shared<typename ActionT::Feedback>();
  if constexpr (std::is_same_v<ActionT, MoveToNamedPose>) {
    (void)phase;
  } else {
    fb->phase = phase;
  }
  fb->progress = progress;
  goal_handle->publish_feedback(fb);
}

template<typename GoalHandleT>
void SkillServer::checkCancel(const GoalHandleT & goal_handle)
{
  if (goal_handle->is_canceling()) {
    move_group_->stop();
    throw SkillFailure(SkillError::CANCELED, "canceled");
  }
}

// Skills

void SkillServer::executeMoveToNamedPose(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamedPose>> gh)
{
  run(
    gh, "MoveToNamedPose", [&](MoveToNamedPose::Result &) {
      const std::string name = gh->get_goal()->name;
      feedback(gh, "", 0.0f);
      moveToNamed(name);
      feedback(gh, "", 1.0f);
    });
}

void SkillServer::executePick(std::shared_ptr<rclcpp_action::ServerGoalHandle<Pick>> gh)
{
  run(
    gh, "Pick", [&](Pick::Result & result) {
      const auto goal = gh->get_goal();
      if (!goal->grasp_strategy.empty() && goal->grasp_strategy != "top") {
        throw SkillFailure(
          SkillError::INVALID_GOAL, "unsupported grasp_strategy '" + goal->grasp_strategy + "'");
      }
      const auto & size = goal->object_size;
      if (size.x <= 0.0 || size.y <= 0.0 || size.z <= 0.0) {
        throw SkillFailure(SkillError::INVALID_GOAL, "object_size must be positive");
      }

      const Eigen::Isometry3d object = toPlanningFrame(goal->object_pose);
      const double bottom = object.translation().z() - 0.5 * size.z;
      const double top = bottom + size.z;
      const double grasp_z = graspHeight(bottom, top, grasp_params_);
      if (std::isnan(grasp_z)) {
        throw SkillFailure(SkillError::INVALID_GOAL, "object is too short to grasp from the top");
      }
      const double yaw = foldYaw(yawOf(object.linear()));
      const Eigen::Vector3d xy(object.translation().x(), object.translation().y(), 0.0);
      const Eigen::Isometry3d grasp = topDownPose(xy + Eigen::Vector3d(0, 0, grasp_z), yaw);
      const Eigen::Isometry3d pregrasp =
        topDownPose(xy + Eigen::Vector3d(0, 0, grasp_z + approach_height_), yaw);

      feedback(gh, "open", 0.0f);
      commandGripper(gripper_open_);
      checkCancel(gh);

      feedback(gh, "approach", 0.15f);
      moveToPose(pregrasp, "pre-grasp");
      checkCancel(gh);

      feedback(gh, "descend", 0.4f);
      moveLinear(grasp, "grasp descent");
      checkCancel(gh);

      feedback(gh, "grasp", 0.6f);
      const double closed = commandGripper(gripper_close_);
      result.gripper_position = closed;
      if (closed > grasp_closed_threshold_) {
        commandGripper(gripper_open_);
        moveLinear(pregrasp, "retreat");
        throw SkillFailure(
          SkillError::GRASP_FAILED,
          fmt("gripper closed to %.3f rad: nothing between the fingers", closed));
      }

      feedback(gh, "lift", 0.75f);
      moveLinear(pregrasp, "lift");

      feedback(gh, "verify", 0.95f);
      const double after_lift = gripperPosition();
      result.gripper_position = after_lift;
      if (after_lift > grasp_closed_threshold_) {
        throw SkillFailure(
          SkillError::GRASP_FAILED, fmt("object slipped out during the lift (gripper %.3f rad)",
          after_lift));
      }
      held_ = HeldObject{goal->object_id, grasp_z - bottom, after_lift};
    });
}

void SkillServer::executePlace(std::shared_ptr<rclcpp_action::ServerGoalHandle<Place>> gh)
{
  run(
    gh, "Place", [&](Place::Result & result) {
      const auto goal = gh->get_goal();
      if (!held_) {
        throw SkillFailure(SkillError::NOT_HOLDING, "not holding an object (no successful Pick)");
      }

      Eigen::Isometry3d target;
      if (!goal->target_name.empty()) {
        geometry_msgs::msg::PoseStamped stamped;
        try {
          stamped.pose = tf2::toMsg(workspace_.placePose(goal->target_name));
        } catch (const std::exception & e) {
          throw SkillFailure(SkillError::UNKNOWN_TARGET, e.what());
        }
        stamped.header.frame_id = workspace_.frame;
        target = toPlanningFrame(stamped);
      } else {
        target = toPlanningFrame(goal->target_pose);
      }
      // use_compliance is not implemented yet: the request is logged and the place goes ahead
      // under position control
      if (goal->use_compliance) {
        RCLCPP_WARN(get_logger(), "use_compliance requested but not available yet; using position control");
      }

      const double yaw = foldYaw(yawOf(target.linear()));
      const Eigen::Vector3d xy(target.translation().x(), target.translation().y(), 0.0);
      const double place_z = target.translation().z() + held_->tcp_above_bottom + release_clearance_;
      const Eigen::Isometry3d place = topDownPose(xy + Eigen::Vector3d(0, 0, place_z), yaw);
      const Eigen::Isometry3d preplace =
        topDownPose(xy + Eigen::Vector3d(0, 0, place_z + approach_height_), yaw);

      feedback(gh, "approach", 0.0f);
      moveToPose(preplace, "pre-place");
      checkCancel(gh);

      feedback(gh, "descend", 0.4f);
      moveLinear(place, "place descent");
      checkCancel(gh);

      feedback(gh, "release", 0.7f);
      const double grip = held_->grip_position;
      const double opened = commandGripper(std::max(gripper_open_, grip - release_opening_));
      if (grip - opened < release_min_opening_) {
        // Leave the pocket or bin before reporting the failure. MoveIt fails every plan while
        // the arm is left inside a collision object, so the next skill could not move at all.
        moveLinear(preplace, "retreat");
        throw SkillFailure(
          SkillError::RELEASE_FAILED,
          fmt("gripper blocked while opening (still at %.3f rad)", opened));
      }
      held_.reset();

      feedback(gh, "retreat", 0.85f);
      moveLinear(preplace, "retreat");
      commandGripper(gripper_open_);

      result.final_tcp_pose.header.frame_id = planning_frame_;
      result.final_tcp_pose.pose = tf2::toMsg(place);
    });
}

// Primitives

void SkillServer::planAndExecute(const std::string & what)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const MoveItErrorCode planned = move_group_->plan(plan);
  if (planned != MoveItErrorCode::SUCCESS) {
    throw SkillFailure(
      SkillError::PLAN_FAILED,
      "no plan to " + what + " (MoveIt error " + std::to_string(planned.val) + ")");
  }
  const MoveItErrorCode executed = move_group_->execute(plan);
  if (executed != MoveItErrorCode::SUCCESS) {
    throw SkillFailure(
      SkillError::EXECUTION_FAILED,
      "moving to " + what + " failed (MoveIt error " + std::to_string(executed.val) + ")");
  }
}

void SkillServer::moveToPose(const Eigen::Isometry3d & tcp_pose, const std::string & what)
{
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = planning_frame_;
  target.pose = tf2::toMsg(tcp_pose);
  move_group_->setStartStateToCurrentState();
  move_group_->setPoseTarget(target, tcp_link_);
  planAndExecute(what);
}

void SkillServer::moveToNamed(const std::string & name)
{
  const auto names = move_group_->getNamedTargets();
  if (std::find(names.begin(), names.end(), name) == names.end()) {
    throw SkillFailure(SkillError::INVALID_GOAL, "unknown named pose '" + name + "'");
  }
  move_group_->setStartStateToCurrentState();
  move_group_->setNamedTarget(name);
  planAndExecute("'" + name + "'");
}

void SkillServer::moveLinear(const Eigen::Isometry3d & tcp_pose, const std::string & what)
{
  // Short vertical moves next to objects and inside pockets are planned without collision
  // checking, because it would flag the intended contact with the object.
  moveit_msgs::msg::RobotTrajectory trajectory;
  move_group_->setStartStateToCurrentState();
  move_group_->setMaxVelocityScalingFactor(cartesian_velocity_scaling_);
  const double fraction = move_group_->computeCartesianPath(
    {tf2::toMsg(tcp_pose)}, 0.002, trajectory, /*avoid_collisions=*/ false);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
  if (fraction < 0.999) {
    throw SkillFailure(
      SkillError::PLAN_FAILED, what + ": Cartesian path " + fmt("%.0f%% complete", 100 * fraction));
  }
  const MoveItErrorCode executed = move_group_->execute(trajectory);
  if (executed != MoveItErrorCode::SUCCESS) {
    throw SkillFailure(
      SkillError::EXECUTION_FAILED,
      what + " failed (MoveIt error " + std::to_string(executed.val) + ")");
  }
}

double SkillServer::commandGripper(double position)
{
  if (!gripper_client_->wait_for_action_server(2s)) {
    throw SkillFailure(SkillError::EXECUTION_FAILED, "gripper action server not available");
  }
  GripperCommand::Goal goal;
  goal.command.name = {gripper_joint_};
  goal.command.position = {position};
  auto goal_future = gripper_client_->async_send_goal(goal);
  if (goal_future.wait_for(2s) != std::future_status::ready || !goal_future.get()) {
    throw SkillFailure(SkillError::EXECUTION_FAILED, "gripper goal was not accepted");
  }
  auto result_future = gripper_client_->async_get_result(goal_future.get());
  if (result_future.wait_for(std::chrono::duration<double>(gripper_timeout_)) !=
    std::future_status::ready)
  {
    gripper_client_->async_cancel_all_goals();
    throw SkillFailure(SkillError::TIMEOUT, "gripper did not finish in time");
  }
  // Closing on an object ends in a stall rather than at the goal, which is expected; the caller
  // checks the position.
  std::this_thread::sleep_for(200ms);
  return gripperPosition();
}

double SkillServer::gripperPosition() const
{
  std::lock_guard<std::mutex> lock(gripper_mutex_);
  if (!gripper_position_) {
    throw SkillFailure(SkillError::EXECUTION_FAILED, "no joint state for " + gripper_joint_);
  }
  return *gripper_position_;
}

Eigen::Isometry3d SkillServer::toPlanningFrame(const geometry_msgs::msg::PoseStamped & pose) const
{
  if (pose.header.frame_id.empty() || pose.header.frame_id == planning_frame_) {
    Eigen::Isometry3d out;
    tf2::fromMsg(pose.pose, out);
    return out;
  }
  try {
    geometry_msgs::msg::PoseStamped in = pose;
    in.header.stamp = rclcpp::Time(0);  // the latest available transform
    const auto transformed = tf_buffer_->transform(in, planning_frame_, tf2::durationFromSec(1.0));
    Eigen::Isometry3d out;
    tf2::fromMsg(transformed.pose, out);
    return out;
  } catch (const tf2::TransformException & e) {
    throw SkillFailure(
      SkillError::INVALID_GOAL,
      "cannot transform from '" + pose.header.frame_id + "' to '" + planning_frame_ + "': " +
      e.what());
  }
}

}  // namespace cell_skills

RCLCPP_COMPONENTS_REGISTER_NODE(cell_skills::SkillServer)
