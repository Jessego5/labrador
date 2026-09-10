// This is the Cartesian compliance controller, a ros2_control admittance controller for a
// position-controlled arm. It reads the joint positions and the wrist force-torque sensor, runs the
// admittance dynamics from admittance.hpp around a reference TCP pose, and commands joint
// positions through a damped least-squares IK step. It commands positions rather than torques
// because the simulated UR5e has position actuators and the sim rejects effort commands, so the
// stiff joint servos stay and it is the commanded pose that is made compliant. It claims the same
// position command interfaces as joint_trajectory_controller, so it is spawned inactive and the
// skills switch the two for the contact-rich motions, compliant placing and insertion.
//
// It has three topics, in the controller's namespace. ~/target_pose (geometry_msgs/PoseStamped)
// is the reference TCP pose in the base frame, followed at a limited speed. ~/current_pose
// (PoseStamped) is the measured TCP pose. ~/wrench (geometry_msgs/WrenchStamped) is the external
// wrench on the tool at the TCP, in the base frame, zeroed at activation and filtered.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cell_controllers/admittance.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace cell_controllers
{

class CartesianComplianceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  Eigen::VectorXd readJoints() const;
  Vector6d readSensor() const;
  // the external wrench on the tool at the TCP, in the base frame, from a raw sensor reading
  Vector6d externalWrench(const Vector6d & raw, const Eigen::VectorXd & q) const;
  void publishState(const rclcpp::Time & time, const Eigen::VectorXd & q);

  // parameters
  std::vector<std::string> joints_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string sensor_frame_;
  std::string ft_sensor_;
  AdmittanceParams admittance_params_;
  double max_force_ = 50.0;
  double max_torque_ = 5.0;
  double force_deadband_ = 0.3;
  double torque_deadband_ = 0.03;
  double filter_cutoff_hz_ = 30.0;
  int tare_samples_ = 50;
  double reference_linear_velocity_ = 0.05;
  double reference_angular_velocity_ = 0.3;
  double max_joint_velocity_ = 1.0;
  int ik_iterations_ = 2;
  double ik_damping_ = 0.02;
  int publish_decimation_ = 2;

  // kinematics
  std::unique_ptr<ChainKinematics> tcp_chain_;
  std::unique_ptr<ChainKinematics> sensor_chain_;

  // control state, touched by the update thread only
  Admittance admittance_;
  Eigen::VectorXd q_cmd_;
  Eigen::Isometry3d reference_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_ = Eigen::Isometry3d::Identity();
  Vector6d bias_ = Vector6d::Zero();
  Vector6d bias_sum_ = Vector6d::Zero();
  Vector6d wrench_filtered_ = Vector6d::Zero();
  int tare_count_ = 0;
  std::size_t cycle_ = 0;
  rclcpp::Time last_target_stamp_;

  // ROS interfaces
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::PoseStamped>> target_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> pose_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>> wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_raw_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_raw_;
};

}  // namespace cell_controllers
