// This is the implementation of the skill server declared in skill_server.hpp: the setup that
// waits for move_group and the controllers, the five skills, and the motion, gripper and
// compliance primitives the skills are built from. Every skill runs in a thread of its own and
// goes through one wrapper, run(), so they all succeed, fail and report in the same way, with a
// SkillError code and a message. Several of the rules in here were learnt in simulation, the
// release that opens only part way, the IK goal instead of a pose goal, the gripper's settle
// check, and the comments next to them say what went wrong.
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
#include "moveit/robot_state/conversions.hpp"
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
  observe_pose_ = param<std::string>("observe_pose", "observe");
  detect_timeout_ = param("detect_timeout", 15.0);
  grasp_params_.grasp_depth = param("grasp_depth", grasp_params_.grasp_depth);
  grasp_params_.fingertip_reach = param("fingertip_reach", grasp_params_.fingertip_reach);
  grasp_params_.surface_clearance = param("surface_clearance", grasp_params_.surface_clearance);
  // for grasp_strategy "insert", hold this far below the top so the rest is free to go in a hole
  insert_grasp_depth_ = param("insert_grasp_depth", 0.012);

  trajectory_controller_ = param<std::string>("trajectory_controller", "joint_trajectory_controller");
  compliance_controller_ = param<std::string>("compliance_controller", "compliance_controller");
  compliance_stiffness_z_ = param("compliance_stiffness_z", 400.0);  // must match the controller
  contact_force_ = param("contact_force", 2.5);          // N, axial force that counts as contact
  press_force_ = param("press_force", 3.0);              // N, held while searching and seating
  compliant_descend_speed_ = param("compliant_descend_speed", 0.01);  // m/s
  compliant_place_height_ = param("compliant_place_height", 0.015);   // m above the place pose
  insert_start_height_ = param("insert_start_height", 0.008);  // peg bottom above the hole, m
  insert_speed_ = param("insert_speed", 0.008);                // m/s
  insert_drop_threshold_ = param("insert_drop_threshold", 0.003);  // m below contact, in the hole
  insert_depth_tolerance_ = param("insert_depth_tolerance", 0.002);  // m short of full depth
  // A stall at press force this close to full depth counts as bottomed out. The depth estimate
  // can be a few mm off when the object shifted in the fingers and no touch-off was possible.
  insert_bottom_tolerance_ = param("insert_bottom_tolerance", 0.004);
  insert_touch_off_window_ = param("insert_touch_off_window", 0.006);
  insert_jam_time_ = param("insert_jam_time", 1.5);            // s without depth progress
  insert_max_force_ = param("insert_max_force", 25.0);         // N
  insert_max_attempts_ = static_cast<int>(param("insert_max_attempts", 3));
  // The peck search probes a grid of points around the expected hole position, lifting, moving
  // and pressing down at each one, because dragging the held peg across the fixture top makes it
  // pivot in the fingers.
  search_spacing_ = param("search_spacing", 0.0015);  // m, below the 2 mm hole clearance window
  search_radius_ = param("search_radius", 0.006);     // m
  search_lift_ = param("search_lift", 0.0015);        // m above the contact height between probes
  search_press_speed_ = param("search_press_speed", 0.02);  // m/s

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
  perception_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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

  detect_client_ = create_client<DetectService>(
    param<std::string>("detect_service", "/perception/detect"), rclcpp::ServicesQoS(),
    perception_group_);

  ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>(
    "/compute_ik", rclcpp::ServicesQoS(), perception_group_);

  // the compliance controller: its state comes in, targets go out, and switching to it goes
  // through the controller manager
  control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller", rclcpp::ServicesQoS(), control_group_);
  const std::string cc = "/" + compliance_controller_;
  compliance_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    cc + "/current_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped & msg) {
      std::lock_guard<std::mutex> lock(compliance_mutex_);
      tf2::fromMsg(msg.pose, compliance_state_.pose);
      compliance_state_.received = std::chrono::steady_clock::now();
      compliance_state_.valid = true;
    },
    sub_options);
  compliance_wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
    cc + "/wrench", 10,
    [this](const geometry_msgs::msg::WrenchStamped & msg) {
      std::lock_guard<std::mutex> lock(compliance_mutex_);
      compliance_state_.force =
        Eigen::Vector3d(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    },
    sub_options);
  compliance_target_pub_ =
    create_publisher<geometry_msgs::msg::PoseStamped>(cc + "/target_pose", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  move_to_named_pose_server_ =
    makeServer<MoveToNamedPose>("~/move_to_named_pose", &SkillServer::executeMoveToNamedPose);
  detect_server_ =
    makeServer<DetectObjects>("~/detect_objects", &SkillServer::executeDetectObjects);
  pick_server_ = makeServer<Pick>("~/pick", &SkillServer::executePick);
  place_server_ = makeServer<Place>("~/place", &SkillServer::executePlace);
  insert_server_ = makeServer<Insert>("~/insert", &SkillServer::executeInsert);

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

  // Free the server before reporting the result. A client may send its next goal as soon as it
  // has the result, and that goal would be rejected as busy if the flag were still set.
  busy_ = false;
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

void SkillServer::executeDetectObjects(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<DetectObjects>> gh)
{
  run(
    gh, "DetectObjects", [&](DetectObjects::Result & result) {
      const auto goal = gh->get_goal();
      // get the arm out of the overhead camera's view first, at the observe pose
      feedback(gh, "move", 0.0f);
      moveToNamed(observe_pose_);
      checkCancel(gh);

      feedback(gh, "capture", 0.6f);
      if (!detect_client_->wait_for_service(2s)) {
        throw SkillFailure(SkillError::PERCEPTION_FAILED, "perception service not available");
      }
      auto request = std::make_shared<DetectService::Request>();
      request->object_class = goal->object_class;
      request->color = goal->color;
      auto future = detect_client_->async_send_request(request);
      if (future.wait_for(std::chrono::duration<double>(detect_timeout_)) !=
        std::future_status::ready)
      {
        detect_client_->remove_pending_request(future);
        throw SkillFailure(SkillError::TIMEOUT, "perception did not answer in time");
      }
      const auto response = future.get();
      if (!response->success) {
        throw SkillFailure(SkillError::PERCEPTION_FAILED, response->message);
      }
      result.objects = response->detections.objects;
      result.message = std::to_string(result.objects.size()) + " object(s)";
    });
}

void SkillServer::executePick(std::shared_ptr<rclcpp_action::ServerGoalHandle<Pick>> gh)
{
  run(
    gh, "Pick", [&](Pick::Result & result) {
      const auto goal = gh->get_goal();
      // "top" is the default top-down grasp. "insert" is top-down too but held near the top, so
      // most of the object is free to go into a hole; Insert checks that it was held high enough.
      GraspParams grasp_params = grasp_params_;
      if (goal->grasp_strategy == "insert") {
        grasp_params.grasp_depth = insert_grasp_depth_;
      } else if (!goal->grasp_strategy.empty() && goal->grasp_strategy != "top") {
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
      const double grasp_z = graspHeight(bottom, top, grasp_params);
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
      const double yaw = foldYaw(yawOf(target.linear()));
      const Eigen::Vector3d xy(target.translation().x(), target.translation().y(), 0.0);
      const double place_z = target.translation().z() + held_->tcp_above_bottom + release_clearance_;
      const Eigen::Isometry3d place = topDownPose(xy + Eigen::Vector3d(0, 0, place_z), yaw);
      const Eigen::Isometry3d preplace =
        topDownPose(xy + Eigen::Vector3d(0, 0, place_z + approach_height_), yaw);

      feedback(gh, "approach", 0.0f);
      moveToPose(preplace, "pre-place");
      checkCancel(gh);

      Eigen::Isometry3d released_at = place;
      double opened = 0.0;
      const double grip = held_->grip_position;
      const double release_to = std::max(gripper_open_, grip - release_opening_);
      if (goal->use_compliance) {
        // Move under position control to just above the target, then descend compliantly until
        // the contact force says the object sits on the target surface, and release there.
        Eigen::Isometry3d hover = place;
        hover.translation().z() += compliant_place_height_;
        feedback(gh, "descend", 0.4f);
        moveLinear(hover, "place descent");
        checkCancel(gh);
        ComplianceScope compliance(*this);
        feedback(gh, "contact", 0.55f);
        const double z_min = place_z - release_clearance_ - 0.01;  // at most 1 cm into the surface
        const ComplianceState s =
          compliantDescend(gh, hover, z_min, compliant_descend_speed_, contact_force_, 10.0);
        released_at = s.pose;
        feedback(gh, "release", 0.7f);
        opened = commandGripper(release_to);
        // lift off the object before switching back, since the fingers are still around it
        Eigen::Isometry3d up = s.pose;
        up.translation().z() += compliant_place_height_;
        sendComplianceTarget(up);
        for (int i = 0; i < 200 && complianceTick(gh).pose.translation().z() <
          up.translation().z() - 0.002; ++i) {}
        compliance.close();
      } else {
        feedback(gh, "descend", 0.4f);
        moveLinear(place, "place descent");
        checkCancel(gh);
        feedback(gh, "release", 0.7f);
        opened = commandGripper(release_to);
      }
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
      result.final_tcp_pose.pose = tf2::toMsg(released_at);
    });
}

void SkillServer::executeInsert(std::shared_ptr<rclcpp_action::ServerGoalHandle<Insert>> gh)
{
  run(
    gh, "Insert", [&](Insert::Result & result) {
      const auto goal = gh->get_goal();
      if (!held_) {
        throw SkillFailure(SkillError::NOT_HOLDING, "not holding an object (no successful Pick)");
      }

      // the hole: the centre of its opening (z up) and how deep the object must go
      Eigen::Isometry3d hole;
      double depth_goal = goal->insert_depth;
      if (!goal->target_name.empty()) {
        const auto it = std::find_if(
          workspace_.fixtures.begin(), workspace_.fixtures.end(),
          [&](const auto & f) {return f.name == goal->target_name;});
        if (it == workspace_.fixtures.end()) {
          throw SkillFailure(SkillError::UNKNOWN_TARGET, "no fixture '" + goal->target_name + "'");
        }
        geometry_msgs::msg::PoseStamped stamped;
        stamped.header.frame_id = workspace_.frame;
        stamped.pose = tf2::toMsg(it->hole_pose);
        hole = toPlanningFrame(stamped);
        depth_goal = it->insert_depth;
      } else {
        hole = toPlanningFrame(goal->target_pose);
      }
      if (depth_goal <= 0.0) {
        throw SkillFailure(SkillError::INVALID_GOAL, "insert_depth must be positive");
      }
      double above = held_->tcp_above_bottom;  // TCP height above the object's bottom
      if (above - grasp_params_.fingertip_reach < depth_goal + grasp_params_.surface_clearance) {
        throw SkillFailure(
          SkillError::INVALID_GOAL,
          fmt("held too low for a %.0f mm insertion (the fingers would hit the fixture); pick with "
          "grasp_strategy 'insert'", 1000 * depth_goal));
      }
      const double max_force = goal->max_force > 0.0f ? goal->max_force : insert_max_force_;
      const int max_attempts = goal->max_attempts > 0 ? goal->max_attempts : insert_max_attempts_;

      const double hole_z = hole.translation().z();
      const double yaw = foldYaw(yawOf(hole.linear()));
      const Eigen::Vector2d nominal = hole.translation().head<2>();
      const auto tcpAt = [&](const Eigen::Vector2d & xy, double z) {
          return topDownPose(Eigen::Vector3d(xy.x(), xy.y(), z), yaw);
        };
      const auto depthOf = [&](const ComplianceState & s) {
          return hole_z - (s.pose.translation().z() - above);
        };
      const double start_z = hole_z + above + insert_start_height_;
      const auto seatedZ = [&]() {return hole_z + above - depth_goal;};  // TCP, object fully in
      const double press_dz = press_force_ / compliance_stiffness_z_;
      double max_seen = 0.0;
      double best_depth = -insert_start_height_;
      const auto progress = [&](const std::string & phase, const ComplianceState & s, float p) {
          auto fb = std::make_shared<Insert::Feedback>();
          fb->phase = phase;
          fb->progress = p;
          fb->depth = static_cast<float>(depthOf(s));
          fb->force = static_cast<float>(s.force.z());
          gh->publish_feedback(fb);
        };

      feedback(gh, "approach", 0.0f);
      moveToPose(tcpAt(nominal, start_z + approach_height_), "pre-insert");
      checkCancel(gh);
      moveLinear(tcpAt(nominal, start_z), "insert approach");
      checkCancel(gh);

      ComplianceScope compliance(*this);
      Eigen::Vector2d center = nominal;  // where to search, moved to where the hole was found
      bool seated = false;
      int attempt = 0;
      std::string last_problem = "no attempt";
      while (!seated && attempt < max_attempts) {
        ++attempt;
        const auto tick = [&]() {
            const ComplianceState s = complianceTick(gh);
            max_seen = std::max(max_seen, s.force.z());
            best_depth = std::max(best_depth, depthOf(s));
            return s;
          };
        const auto backOff = [&](const Eigen::Vector2d & xy) {
            sendComplianceTarget(tcpAt(xy, start_z));
            for (int i = 0; i < 300 && tick().pose.translation().z() < start_z - 0.002; ++i) {}
          };

        // step 1: descend until contact, or straight into the hole
        progress("contact", complianceState(), 0.2f);
        // Every attempt searches around the nominal pose, because a failed attempt's "found"
        // position may be where the object caught on the rim rather than the hole.
        center = nominal;
        ComplianceState s = compliantDescend(
          gh, tcpAt(center, start_z), seatedZ() - press_dz, compliant_descend_speed_,
          contact_force_, 10.0);
        max_seen = std::max(max_seen, s.force.z());
        // Touch-off. Contact near the expected height is the fixture's top face, whose height is
        // known, so it tells exactly how far the object reaches below the TCP. The estimate from
        // the grasp is a few mm off when closing the fingers shifted the object.
        if (s.force.z() > contact_force_ && std::abs(depthOf(s)) < insert_touch_off_window_) {
          const double measured = s.pose.translation().z() - hole_z;
          RCLCPP_INFO(
            get_logger(), "Insert touch-off: object reaches %.1f mm below the TCP (grasp estimate "
            "%.1f mm)", 1000 * measured, 1000 * above);
          above = measured;
        }
        bool entered = depthOf(s) > insert_drop_threshold_;

        // Step 2: after landing beside the hole, a peck search around the expected position.
        // Each probe lifts off, moves and presses down again. Dragging the tip across the
        // surface instead makes the part pivot in the fingers; a dragging spiral search failed
        // that way, with the peg's tip lagging the gripper by up to 6 mm.
        if (!entered) {
          const double contact_z = s.pose.translation().z();
          const double contact_depth = depthOf(s);
          const double lift_z = contact_z + search_lift_;
          const auto pattern = peckSearchPattern(search_spacing_, search_radius_);
          for (std::size_t k = 1; k < pattern.size() && !entered; ++k) {  // [0] was the descent
            const Eigen::Vector2d xy = center + pattern[k];
            sendComplianceTarget(tcpAt(s.pose.translation().head<2>(), lift_z));
            for (int i = 0; i < 50 && s.pose.translation().z() < lift_z - 0.0005; ++i) {
              s = tick();
            }
            sendComplianceTarget(tcpAt(xy, lift_z));
            for (int i = 0; i < 50 && (s.pose.translation().head<2>() - xy).norm() > 0.0003; ++i) {
              s = tick();
            }
            s = compliantDescend(
              gh, tcpAt(xy, lift_z), contact_z - press_dz, search_press_speed_, contact_force_, 1.0);
            max_seen = std::max(max_seen, s.force.z());
            best_depth = std::max(best_depth, depthOf(s));
            entered = depthOf(s) - contact_depth > insert_drop_threshold_;
            if (k % 5 == 0) {
              progress("search", s, 0.3f);
            }
          }
          if (!entered) {
            last_problem = fmt("hole not found within %.1f mm", 1000 * search_radius_);
            RCLCPP_WARN(get_logger(), "Insert attempt %d: %s", attempt, last_problem.c_str());
            backOff(center);
            continue;
          }
          RCLCPP_INFO(
            get_logger(), "Insert: hole found at %+.1f, %+.1f mm from the expected position",
            1000 * (s.pose.translation().x() - nominal.x()),
            1000 * (s.pose.translation().y() - nominal.y()));
        }

        // step 3: in the hole, stop searching here and push down to full depth
        const Eigen::Vector2d found = s.pose.translation().head<2>();
        center = found;
        ProgressMonitor monitor(0.0005, insert_jam_time_);
        const auto t0 = std::chrono::steady_clock::now();
        const auto now = [&]() {
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
          };
        monitor.reset(0.0, depthOf(s));
        double z_ref = s.pose.translation().z();
        int n = 0;
        while (true) {
          z_ref = std::max(z_ref - insert_speed_ * 0.01, seatedZ() - press_dz);
          sendComplianceTarget(tcpAt(found, z_ref));
          s = tick();
          const double depth = depthOf(s);
          if (depth >= depth_goal - insert_depth_tolerance_) {
            seated = true;
            break;
          }
          const bool stalled = monitor.update(now(), depth);
          if (stalled && depth >= depth_goal - insert_bottom_tolerance_ &&
            s.force.z() > 0.8 * press_force_ && s.force.z() < max_force)
          {
            RCLCPP_INFO(get_logger(), "Insert: bottomed out at %.1f mm estimated depth", 1000 * depth);
            seated = true;
            break;
          }
          if (stalled || s.force.z() > max_force) {
            last_problem = fmt("jammed at %.1f mm depth", 1000 * depth) +
              fmt(" (axial force %.1f N)", s.force.z());
            RCLCPP_WARN(get_logger(), "Insert attempt %d: %s", attempt, last_problem.c_str());
            break;
          }
          if (++n % 10 == 0) {
            progress("insert", s, 0.6f);
          }
        }
        if (seated) {
          center = s.pose.translation().head<2>();  // object axis = hole axis, within clearance
        } else {
          backOff(found);
        }
      }
      result.attempts = static_cast<uint8_t>(attempt);
      result.depth_reached = static_cast<float>(std::max(0.0, best_depth));
      result.max_force_seen = static_cast<float>(max_seen);
      result.hole_offset.x = center.x() - nominal.x();
      result.hole_offset.y = center.y() - nominal.y();
      if (!seated) {
        compliance.close();
        moveLinear(tcpAt(center, start_z + approach_height_), "retreat");
        throw SkillFailure(
          SkillError::INSERT_JAMMED,
          "insertion failed after " + std::to_string(attempt) + " attempt(s): " + last_problem);
      }

      // step 4: release in the hole and lift the fingers off before switching back
      ComplianceState s = complianceState();
      progress("release", s, 0.85f);
      const double grip = held_->grip_position;
      const double opened = commandGripper(std::max(gripper_open_, grip - release_opening_));
      if (grip - opened < release_min_opening_) {
        throw SkillFailure(
          SkillError::RELEASE_FAILED,
          fmt("gripper blocked while opening (still at %.3f rad)", opened));
      }
      held_.reset();
      sendComplianceTarget(tcpAt(center, start_z));
      for (int i = 0; i < 300 && complianceTick(gh).pose.translation().z() < start_z - 0.002; ++i) {}
      compliance.close();

      feedback(gh, "retreat", 0.95f);
      moveLinear(tcpAt(center, start_z + approach_height_), "retreat");
      commandGripper(gripper_open_);
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
  // Plan to a collision-free IK solution seeded from the current joints rather than to the pose
  // itself. Given a pose goal, OMPL samples its own IK and occasionally commits to an awkward
  // configuration it cannot connect within the planning time; in the part 4 benchmark that caused 3
  // failures out of 50. Without an IK solution the skill falls back to the pose goal.
  std::map<std::string, double> joints;
  if (solveIk(target, joints)) {
    std::string values;
    for (const auto & [name, value] : joints) {
      values += " " + name.substr(0, name.find("_joint")) + "=" + fmt("%.2f", value);
    }
    RCLCPP_DEBUG(get_logger(), "%s: planning to IK solution%s", what.c_str(), values.c_str());
    move_group_->setJointValueTarget(joints);
  } else {
    RCLCPP_WARN(get_logger(), "%s: no collision-free IK solution; planning to the pose", what.c_str());
    move_group_->setPoseTarget(target, tcp_link_);
  }
  planAndExecute(what);
}

bool SkillServer::solveIk(
  const geometry_msgs::msg::PoseStamped & target, std::map<std::string, double> & joints)
{
  if (!ik_client_->wait_for_service(1s)) {
    return false;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
  request->ik_request.group_name = arm_group_;
  request->ik_request.ik_link_name = tcp_link_;
  request->ik_request.pose_stamped = target;
  request->ik_request.avoid_collisions = true;
  request->ik_request.timeout = rclcpp::Duration::from_seconds(0.2);
  const auto current = move_group_->getCurrentState(1.0);
  if (!current) {
    return false;
  }
  moveit::core::robotStateToRobotStateMsg(*current, request->ik_request.robot_state);
  auto future = ik_client_->async_send_request(request);
  if (future.wait_for(3s) != std::future_status::ready) {
    ik_client_->remove_pending_request(future);
    return false;
  }
  const auto response = future.get();
  if (response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    return false;
  }
  const auto active = move_group_->getActiveJoints();
  const auto & js = response->solution.joint_state;
  for (std::size_t i = 0; i < js.name.size() && i < js.position.size(); ++i) {
    if (std::find(active.begin(), active.end(), js.name[i]) != active.end()) {
      joints[js.name[i]] = js.position[i];
    }
  }
  return joints.size() == active.size();
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
  // Done when the controller reports a result, or when the fingers have stopped moving for
  // 0.3 s. Closing on a thin part such as the peg, contact chatter keeps the measured velocity
  // above the controller's stall threshold and defeats its stall check. In that case the goal
  // stays active, with the fingers still squeezing, until the next command preempts it. The
  // caller checks the position either way.
  const auto start = std::chrono::steady_clock::now();
  const auto deadline = start + std::chrono::duration<double>(gripper_timeout_);
  constexpr auto settle_window = 300ms;
  constexpr double settle_tolerance = 0.003;  // rad over the window
  auto window_start = start;
  double window_position = gripperPosition();
  while (result_future.wait_for(20ms) != std::future_status::ready) {
    const auto now = std::chrono::steady_clock::now();
    const double q = gripperPosition();
    if (std::abs(q - window_position) > settle_tolerance) {
      window_start = now;
      window_position = q;
    } else if (now - window_start > settle_window && std::abs(q - position) > 0.02 &&
      now - start > 500ms)
    {
      RCLCPP_DEBUG(get_logger(), "Gripper settled at %.3f rad (goal %.3f)", q, position);
      return q;  // stalled on an object
    }
    if (now > deadline) {
      gripper_client_->async_cancel_all_goals();
      throw SkillFailure(SkillError::TIMEOUT, "gripper did not finish in time");
    }
  }
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

// Compliance

SkillServer::ComplianceScope::ComplianceScope(SkillServer & server)
: server_(server)
{
  server_.switchControllers({server_.compliance_controller_}, {server_.trajectory_controller_});
  active_ = true;
  // The controller zeroes the force-torque sensor during its first cycles, so wait for that and
  // for a fresh state before using it.
  std::this_thread::sleep_for(400ms);
  const auto s = server_.complianceState();
  if (!s.valid || std::chrono::steady_clock::now() - s.received > 200ms) {
    close();
    throw SkillFailure(SkillError::EXECUTION_FAILED, "no state from the compliance controller");
  }
}

SkillServer::ComplianceScope::~ComplianceScope()
{
  try {
    close();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(server_.get_logger(), "Switching back from compliance failed: %s", e.what());
  }
}

void SkillServer::ComplianceScope::close()
{
  if (active_) {
    active_ = false;
    server_.switchControllers({server_.trajectory_controller_}, {server_.compliance_controller_});
  }
}

void SkillServer::switchControllers(
  const std::vector<std::string> & start, const std::vector<std::string> & stop)
{
  using controller_manager_msgs::srv::SwitchController;
  if (!switch_client_->wait_for_service(2s)) {
    throw SkillFailure(SkillError::EXECUTION_FAILED, "controller manager not available");
  }
  auto request = std::make_shared<SwitchController::Request>();
  request->activate_controllers = start;
  request->deactivate_controllers = stop;
  request->strictness = SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration::from_seconds(2.0);
  auto future = switch_client_->async_send_request(request);
  if (future.wait_for(5s) != std::future_status::ready) {
    switch_client_->remove_pending_request(future);
    throw SkillFailure(SkillError::TIMEOUT, "controller switch timed out");
  }
  if (!future.get()->ok) {
    throw SkillFailure(
      SkillError::EXECUTION_FAILED,
      "controller switch to [" + (start.empty() ? std::string() : start.front()) + "] failed");
  }
  RCLCPP_DEBUG(get_logger(), "Switched to %s", start.empty() ? "-" : start.front().c_str());
}

SkillServer::ComplianceState SkillServer::complianceState() const
{
  std::lock_guard<std::mutex> lock(compliance_mutex_);
  return compliance_state_;
}

template<typename GoalHandleT>
SkillServer::ComplianceState SkillServer::complianceTick(const GoalHandleT & goal_handle)
{
  std::this_thread::sleep_for(10ms);
  if (goal_handle->is_canceling()) {
    // hold where the arm is, the ComplianceScope switches back on the way out
    sendComplianceTarget(complianceState().pose);
    throw SkillFailure(SkillError::CANCELED, "canceled");
  }
  const ComplianceState s = complianceState();
  if (!s.valid || std::chrono::steady_clock::now() - s.received > 500ms) {
    throw SkillFailure(SkillError::EXECUTION_FAILED, "compliance controller state is stale");
  }
  return s;
}

void SkillServer::sendComplianceTarget(const Eigen::Isometry3d & tcp_pose)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = planning_frame_;
  msg.header.stamp = now();
  msg.pose = tf2::toMsg(tcp_pose);
  compliance_target_pub_->publish(msg);
}

template<typename GoalHandleT>
SkillServer::ComplianceState SkillServer::compliantDescend(
  const GoalHandleT & goal_handle, const Eigen::Isometry3d & pose, double z_min, double speed,
  double force, double timeout)
{
  Eigen::Isometry3d target = pose;
  const double dt = 0.01;
  const int max_ticks = static_cast<int>(timeout / dt);
  ComplianceState s = complianceState();
  for (int i = 0; i < max_ticks; ++i) {
    target.translation().z() = std::max(target.translation().z() - speed * dt, z_min);
    sendComplianceTarget(target);
    s = complianceTick(goal_handle);
    if (s.force.z() > force) {
      return s;
    }
    // reached the lowest allowed height and settled there, without contact
    if (target.translation().z() <= z_min && s.pose.translation().z() < z_min + 0.001) {
      return s;
    }
  }
  return s;
}

}  // namespace cell_skills

RCLCPP_COMPONENTS_REGISTER_NODE(cell_skills::SkillServer)
