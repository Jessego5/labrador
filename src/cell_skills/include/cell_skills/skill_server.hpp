// This is the skill server, the node that offers the cell's skills as ROS 2 actions under
// /skills/: MoveToNamedPose, DetectObjects, Pick, Place and Insert. The task executor and the
// trial scripts are its clients, and cell.launch.py starts it next to move_group. All five live
// in one node so they share one MoveGroupInterface and the record of the held object that Pick
// hands on to Place and Insert. Only one skill runs at a time, and a goal that arrives while
// another is running is rejected rather than queued. The contact phases, a compliant place and
// an insertion, switch from the joint trajectory controller to the Cartesian compliance
// controller and back again when they are done.
#pragma once

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "cell_interfaces/action/detect_objects.hpp"
#include "cell_interfaces/action/insert.hpp"
#include "cell_interfaces/action/move_to_named_pose.hpp"
#include "cell_interfaces/action/pick.hpp"
#include "cell_interfaces/action/place.hpp"
#include "cell_interfaces/msg/skill_error.hpp"
#include "cell_interfaces/srv/detect_objects.hpp"
#include "cell_skills/grasp_geometry.hpp"
#include "cell_skills/insertion.hpp"
#include "cell_workspace/workspace.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
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
  using DetectObjects = cell_interfaces::action::DetectObjects;
  using DetectService = cell_interfaces::srv::DetectObjects;
  using Pick = cell_interfaces::action::Pick;
  using Place = cell_interfaces::action::Place;
  using Insert = cell_interfaces::action::Insert;
  using SkillError = cell_interfaces::msg::SkillError;
  using GripperCommand = control_msgs::action::ParallelGripperCommand;

  explicit SkillServer(const rclcpp::NodeOptions & options);

private:
  // the latest state reported by the compliance controller
  struct ComplianceState
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // measured TCP, planning frame
    Eigen::Vector3d force = Eigen::Vector3d::Zero();         // external force on the tool
    std::chrono::steady_clock::time_point received;
    bool valid = false;
  };

  // Keeps the compliance controller active for the lifetime of the scope. It switches back to
  // the trajectory controller on the way out, also when a skill throws.
  class ComplianceScope
  {
  public:
    explicit ComplianceScope(SkillServer & server);
    ~ComplianceScope();
    void close();

  private:
    SkillServer & server_;
    bool active_ = false;
  };

  // what Pick remembers for Place and Insert
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
  void executeDetectObjects(std::shared_ptr<rclcpp_action::ServerGoalHandle<DetectObjects>> gh);
  void executePick(std::shared_ptr<rclcpp_action::ServerGoalHandle<Pick>> gh);
  void executePlace(std::shared_ptr<rclcpp_action::ServerGoalHandle<Place>> gh);
  void executeInsert(std::shared_ptr<rclcpp_action::ServerGoalHandle<Insert>> gh);

  // motion and gripper primitives, which throw SkillFailure
  void planAndExecute(const std::string & what);
  void moveToPose(const Eigen::Isometry3d & tcp_pose, const std::string & what);
  // a collision-free IK solution for `target`, seeded from the current joints or from `seed`
  bool solveIk(
    const geometry_msgs::msg::PoseStamped & target, std::map<std::string, double> & joints,
    const std::map<std::string, double> & seed = {});
  // moves to `above` on the IK branch of `below`, so that the straight line down between them
  // stays on one branch
  void moveAboveOnBranchOf(
    const Eigen::Isometry3d & above, const Eigen::Isometry3d & below, const std::string & what);
  void moveToNamed(const std::string & name);
  void moveLinear(const Eigen::Isometry3d & tcp_pose, const std::string & what);
  double commandGripper(double position);
  double gripperPosition() const;
  Eigen::Isometry3d toPlanningFrame(const geometry_msgs::msg::PoseStamped & pose) const;

  // compliance primitives, which throw SkillFailure
  void switchControllers(const std::vector<std::string> & start, const std::vector<std::string> & stop);
  ComplianceState complianceState() const;
  // a fresh compliance state, after waiting one control tick (about 10 ms)
  template<typename GoalHandleT>
  ComplianceState complianceTick(const GoalHandleT & goal_handle);
  void sendComplianceTarget(const Eigen::Isometry3d & tcp_pose);
  // The compliance state at the end of a descent that lowers the TCP at `speed`, keeping the x,
  // y and orientation of `pose`, until the axial contact force exceeds `force` or the TCP
  // reaches `z_min`.
  template<typename GoalHandleT>
  ComplianceState compliantDescend(
    const GoalHandleT & goal_handle, const Eigen::Isometry3d & pose, double z_min, double speed,
    double force, double timeout);

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
  double compliant_release_lift_;  // m above contact to hold while opening (compliant place)
  double gripper_timeout_;
  std::string observe_pose_;
  double detect_timeout_;
  GraspParams grasp_params_;
  double insert_grasp_depth_;
  // compliance
  std::string trajectory_controller_;
  std::string compliance_controller_;
  double compliance_stiffness_z_;
  double contact_force_;
  double press_force_;
  double compliant_descend_speed_;
  double compliant_place_height_;
  // insertion
  double insert_start_height_;
  double insert_speed_;
  double insert_drop_threshold_;
  double insert_depth_tolerance_;
  double insert_bottom_tolerance_;
  double insert_touch_off_window_;
  double insert_jam_time_;
  double insert_max_force_;
  int insert_max_attempts_;
  double search_spacing_;
  double search_radius_;
  double search_lift_;
  double search_press_speed_;

  // state
  cell_workspace::Workspace workspace_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::string planning_frame_;
  std::atomic<bool> ready_{false};
  std::atomic<bool> busy_{false};
  std::optional<HeldObject> held_;
  mutable std::mutex gripper_mutex_;
  std::optional<double> gripper_position_;
  mutable std::mutex compliance_mutex_;
  ComplianceState compliance_state_;

  // ROS interfaces
  rclcpp::CallbackGroup::SharedPtr server_group_;
  rclcpp::CallbackGroup::SharedPtr gripper_group_;
  rclcpp::CallbackGroup::SharedPtr state_group_;
  rclcpp::CallbackGroup::SharedPtr perception_group_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::Client<DetectService>::SharedPtr detect_client_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr compliance_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr compliance_wrench_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr compliance_target_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Server<MoveToNamedPose>::SharedPtr move_to_named_pose_server_;
  rclcpp_action::Server<DetectObjects>::SharedPtr detect_server_;
  rclcpp_action::Server<Pick>::SharedPtr pick_server_;
  rclcpp_action::Server<Place>::SharedPtr place_server_;
  rclcpp_action::Server<Insert>::SharedPtr insert_server_;
};

}  // namespace cell_skills
