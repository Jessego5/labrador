// This is the skill server, the node that offers the cell's skills as ROS 2 actions under
// /skills/: MoveToNamedPose, Pick and Place. run_pick_place_trials.py is its client, and
// cell.launch.py starts it next to move_group. All three live in one node so they share one
// MoveGroupInterface and the record of the held object that Pick hands on to Place. Only one
// skill runs at a time, and a goal that arrives while another is running is rejected rather than
// queued.
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include "cell_interfaces/action/move_to_named_pose.hpp"
#include "cell_interfaces/action/pick.hpp"
#include "cell_interfaces/action/place.hpp"
#include "cell_interfaces/msg/skill_error.hpp"
#include "cell_skills/grasp_geometry.hpp"
#include "cell_workspace/workspace.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace cell_skills
{

// thrown inside a skill to end it with a specific SkillError code
class SkillFailure : public std::runtime_error
{
public:
  SkillFailure(uint8_t code, const std::string & message)
  : std::runtime_error(message), code(code) {}
  uint8_t code;
};

class SkillServer : public rclcpp::Node
{
public:
  using MoveToNamedPose = cell_interfaces::action::MoveToNamedPose;
  using Pick = cell_interfaces::action::Pick;
  using Place = cell_interfaces::action::Place;
  using SkillError = cell_interfaces::msg::SkillError;
  using GripperCommand = control_msgs::action::ParallelGripperCommand;

  explicit SkillServer(const rclcpp::NodeOptions & options);

private:
  // what Pick remembers for Place
  struct HeldObject
  {
    std::string id;
    double tcp_above_bottom;  // TCP height above the object's bottom face
    double grip_position;     // gripper knuckle angle while holding it
  };

  // setup
  template<typename T>
  T param(const std::string & name, const T & default_value);
  void initialize();
  void addCollisionObjects();

  template<typename ActionT>
  typename rclcpp_action::Server<ActionT>::SharedPtr makeServer(
    const std::string & name,
    void (SkillServer::* execute)(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>));

  // the wrapper every skill runs in: readiness check, result and error reporting, logging and
  // the busy flag
  template<typename ActionT, typename Body>
  void run(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
    const std::string & skill, Body && body);

  template<typename ActionT>
  void feedback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
    const std::string & phase, float progress);

  template<typename GoalHandleT>
  void checkCancel(const GoalHandleT & goal_handle);

  // skills
  void executeMoveToNamedPose(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamedPose>> gh);
  void executePick(std::shared_ptr<rclcpp_action::ServerGoalHandle<Pick>> gh);
  void executePlace(std::shared_ptr<rclcpp_action::ServerGoalHandle<Place>> gh);

  // motion and gripper primitives, which throw SkillFailure
  void planAndExecute(const std::string & what);
  void moveToPose(const Eigen::Isometry3d & tcp_pose, const std::string & what);
  void moveToNamed(const std::string & name);
  void moveLinear(const Eigen::Isometry3d & tcp_pose, const std::string & what);
  double commandGripper(double position);
  double gripperPosition() const;
  Eigen::Isometry3d toPlanningFrame(const geometry_msgs::msg::PoseStamped & pose) const;

  // parameters
  std::string arm_group_;
  std::string tcp_link_;
  std::string gripper_joint_;
  double velocity_scaling_;
  double acceleration_scaling_;
  double cartesian_velocity_scaling_;
  double approach_height_;
  double release_clearance_;
  double gripper_open_;
  double gripper_close_;
  double grasp_closed_threshold_;
  double release_opening_;
  double release_min_opening_;
  double gripper_timeout_;
  GraspParams grasp_params_;

  // state
  cell_workspace::Workspace workspace_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string planning_frame_;
  std::atomic<bool> ready_{false};
  std::atomic<bool> busy_{false};
  std::optional<HeldObject> held_;
  mutable std::mutex gripper_mutex_;
  std::optional<double> gripper_position_;

  // ROS interfaces
  rclcpp::CallbackGroup::SharedPtr server_group_;
  rclcpp::CallbackGroup::SharedPtr gripper_group_;
  rclcpp::CallbackGroup::SharedPtr state_group_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Server<MoveToNamedPose>::SharedPtr move_to_named_pose_server_;
  rclcpp_action::Server<Pick>::SharedPtr pick_server_;
  rclcpp_action::Server<Place>::SharedPtr place_server_;
};

}  // namespace cell_skills
