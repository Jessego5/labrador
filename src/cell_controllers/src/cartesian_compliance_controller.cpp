// This is the ros2_control side of the compliance controller: parameters, interfaces, the
// realtime topics and the update loop that ties the admittance math to the arm. Each cycle it
// zeroes the force-torque sensor for the first tare_samples cycles after activation, then moves
// the reference towards the latest ~/target_pose at a limited speed, turns the raw sensor reading
// into a filtered external wrench at the TCP, steps the admittance and commands the joints through
// one IK step. The tuned parameters live in cell_description/config/ros2_controllers.yaml under
// compliance_controller; the values declared here are only fallbacks.
#include "cell_controllers/cartesian_compliance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace cell_controllers
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

namespace
{

constexpr const char * kSensorInterfaces[6] = {
  "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};

Vector6d toVector6(const std::vector<double> & v, const std::string & name)
{
  if (v.size() != 6) {
    throw std::runtime_error("parameter '" + name + "' needs 6 values");
  }
  Vector6d out;
  for (int i = 0; i < 6; ++i) {
    out[i] = v[static_cast<std::size_t>(i)];
  }
  return out;
}

Eigen::Vector3d toVector3(const std::vector<double> & v, const std::string & name)
{
  if (v.size() != 3) {
    throw std::runtime_error("parameter '" + name + "' needs 3 values");
  }
  return {v[0], v[1], v[2]};
}

void toMsg(const Eigen::Isometry3d & pose, geometry_msgs::msg::Pose & msg)
{
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  msg.orientation.x = q.x();
  msg.orientation.y = q.y();
  msg.orientation.z = q.z();
  msg.orientation.w = q.w();
}

Eigen::Isometry3d fromMsg(const geometry_msgs::msg::Pose & msg)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
  out.linear() = Eigen::Quaterniond(
    msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
    .normalized().toRotationMatrix();
  return out;
}

}  // namespace

CallbackReturn CartesianComplianceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("base_frame", "world");
    auto_declare<std::string>("tip_frame", "tcp");
    auto_declare<std::string>("sensor_frame", "tool0");
    auto_declare<std::string>("ft_sensor", "wrist_ft_fts");
    auto_declare<std::vector<double>>("stiffness", {400, 400, 400, 30, 30, 30});
    auto_declare<std::vector<double>>("mass", {2, 2, 2, 0.05, 0.05, 0.05});
    auto_declare<std::vector<double>>("damping", {});
    auto_declare<double>("damping_ratio", 1.0);
    auto_declare<double>("max_force", max_force_);
    auto_declare<double>("max_torque", max_torque_);
    auto_declare<double>("force_deadband", force_deadband_);
    auto_declare<double>("torque_deadband", torque_deadband_);
    auto_declare<double>("filter_cutoff_hz", filter_cutoff_hz_);
    auto_declare<int>("tare_samples", tare_samples_);
    auto_declare<double>("max_linear_velocity", 0.05);
    auto_declare<double>("max_angular_velocity", 0.3);
    auto_declare<double>("reference_linear_velocity", reference_linear_velocity_);
    auto_declare<double>("reference_angular_velocity", reference_angular_velocity_);
    auto_declare<double>("max_joint_velocity", max_joint_velocity_);
    auto_declare<std::vector<double>>("workspace_min", {-2.0, -2.0, 0.0});
    auto_declare<std::vector<double>>("workspace_max", {2.0, 2.0, 2.0});
    auto_declare<int>("ik_iterations", ik_iterations_);
    auto_declare<double>("ik_damping", ik_damping_);
    auto_declare<int>("publish_decimation", publish_decimation_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration CartesianComplianceController::command_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {
    config.names.push_back(j + "/position");
  }
  return config;
}

InterfaceConfiguration CartesianComplianceController::state_interface_configuration() const
{
  InterfaceConfiguration config{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {
    config.names.push_back(j + "/position");
  }
  for (const char * name : kSensorInterfaces) {
    config.names.push_back(ft_sensor_ + "/" + name);
  }
  return config;
}

CallbackReturn CartesianComplianceController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  try {
    joints_ = node->get_parameter("joints").as_string_array();
    base_frame_ = node->get_parameter("base_frame").as_string();
    tip_frame_ = node->get_parameter("tip_frame").as_string();
    sensor_frame_ = node->get_parameter("sensor_frame").as_string();
    ft_sensor_ = node->get_parameter("ft_sensor").as_string();

    AdmittanceParams p;
    p.stiffness = toVector6(node->get_parameter("stiffness").as_double_array(), "stiffness");
    p.mass = toVector6(node->get_parameter("mass").as_double_array(), "mass");
    // explicit damping if given, otherwise from the damping ratio (relative to K and M alone)
    const auto damping = node->get_parameter("damping").as_double_array();
    p.damping = damping.empty() ?
      AdmittanceParams::criticalDamping(
      p.mass, p.stiffness, node->get_parameter("damping_ratio").as_double()) :
      toVector6(damping, "damping");
    p.max_linear_velocity = node->get_parameter("max_linear_velocity").as_double();
    p.max_angular_velocity = node->get_parameter("max_angular_velocity").as_double();
    p.workspace_min = toVector3(node->get_parameter("workspace_min").as_double_array(), "workspace_min");
    p.workspace_max = toVector3(node->get_parameter("workspace_max").as_double_array(), "workspace_max");
    if ((p.mass.array() <= 0).any() || (p.stiffness.array() < 0).any()) {
      throw std::runtime_error("mass must be positive and stiffness non-negative");
    }
    admittance_params_ = p;
    admittance_.setParams(p);

    max_force_ = node->get_parameter("max_force").as_double();
    max_torque_ = node->get_parameter("max_torque").as_double();
    force_deadband_ = node->get_parameter("force_deadband").as_double();
    torque_deadband_ = node->get_parameter("torque_deadband").as_double();
    filter_cutoff_hz_ = node->get_parameter("filter_cutoff_hz").as_double();
    tare_samples_ = std::max(1, static_cast<int>(node->get_parameter("tare_samples").as_int()));
    reference_linear_velocity_ = node->get_parameter("reference_linear_velocity").as_double();
    reference_angular_velocity_ = node->get_parameter("reference_angular_velocity").as_double();
    max_joint_velocity_ = node->get_parameter("max_joint_velocity").as_double();
    ik_iterations_ = std::max(1, static_cast<int>(node->get_parameter("ik_iterations").as_int()));
    ik_damping_ = node->get_parameter("ik_damping").as_double();
    publish_decimation_ =
      std::max(1, static_cast<int>(node->get_parameter("publish_decimation").as_int()));

    const std::string urdf = get_robot_description();
    if (urdf.empty()) {
      throw std::runtime_error("no robot description");
    }
    tcp_chain_ = std::make_unique<ChainKinematics>(urdf, base_frame_, tip_frame_);
    sensor_chain_ = std::make_unique<ChainKinematics>(urdf, base_frame_, sensor_frame_);
    if (tcp_chain_->jointNames() != joints_ || sensor_chain_->jointNames() != joints_) {
      throw std::runtime_error(
              "the chain " + base_frame_ + " -> " + tip_frame_ + " (and -> " + sensor_frame_ +
              ") must contain exactly the configured joints, in order");
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "on_configure: %s", e.what());
    return CallbackReturn::ERROR;
  }

  target_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_pose", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
      if (!msg->header.frame_id.empty() && msg->header.frame_id != base_frame_) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "ignoring target in frame '%s' (expected '%s')", msg->header.frame_id.c_str(),
          base_frame_.c_str());
        return;
      }
      target_buffer_.writeFromNonRT(msg);
    });
  pose_pub_raw_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/current_pose", rclcpp::SystemDefaultsQoS());
  wrench_pub_raw_ = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "~/wrench", rclcpp::SystemDefaultsQoS());
  pose_pub_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
    pose_pub_raw_);
  wrench_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
    wrench_pub_raw_);

  RCLCPP_INFO(
    node->get_logger(), "Configured: %zu joints, %s -> %s, sensor '%s' at %s",
    joints_.size(), base_frame_.c_str(), tip_frame_.c_str(), ft_sensor_.c_str(),
    sensor_frame_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianComplianceController::on_activate(const rclcpp_lifecycle::State &)
{
  if (command_interfaces_.size() != joints_.size() ||
    state_interfaces_.size() != joints_.size() + 6)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "on_activate: missing interfaces");
    return CallbackReturn::ERROR;
  }
  // hold where the arm is, and zero the sensor (gripper and payload weight) before going
  // compliant
  q_cmd_ = readJoints();
  const Eigen::Isometry3d here = tcp_chain_->fk(q_cmd_);
  admittance_.reset(here);
  reference_ = here;
  target_ = here;
  target_buffer_.writeFromNonRT(nullptr);
  bias_.setZero();
  bias_sum_.setZero();
  wrench_filtered_.setZero();
  tare_count_ = 0;
  cycle_ = 0;
  RCLCPP_INFO(get_node()->get_logger(), "Activated; zeroing the force-torque sensor");
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianComplianceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

Eigen::VectorXd CartesianComplianceController::readJoints() const
{
  Eigen::VectorXd q(static_cast<Eigen::Index>(joints_.size()));
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    q[static_cast<Eigen::Index>(i)] =
      state_interfaces_[i].get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
  }
  return q;
}

Vector6d CartesianComplianceController::readSensor() const
{
  Vector6d raw;
  for (std::size_t i = 0; i < 6; ++i) {
    raw[static_cast<Eigen::Index>(i)] = state_interfaces_[joints_.size() + i].get_optional().value_or(
      std::numeric_limits<double>::quiet_NaN());
  }
  return raw;
}

Vector6d CartesianComplianceController::externalWrench(
  const Vector6d & raw, const Eigen::VectorXd & q) const
{
  // The sensor, as mujoco_ros2_control exports it, reports the wrench the tool side applies at
  // the sensor, in the sensor frame: the gripper's weight plus whatever the environment does to
  // the tool. mujoco_ros2_control negates MuJoCo's own reading to get this, and the sign matters:
  // getting it wrong gave positive feedback (-60 N), with the arm pushing harder the more it was
  // resisted. With the weight zeroed at activation, the change is the external wrench.
  const Eigen::Isometry3d sensor = sensor_chain_->fk(q);
  const Eigen::Isometry3d tcp = tcp_chain_->fk(q);
  const Eigen::Matrix3d R = sensor.linear();
  const Vector6d d = raw - bias_;
  Vector6d w;
  w.head<3>() = R * d.head<3>();
  // the moment about the TCP is the moment about the sensor plus (p_sensor - p_tcp) x F
  w.tail<3>() = R * d.tail<3>() +
    (sensor.translation() - tcp.translation()).cross(Eigen::Vector3d(w.head<3>()));
  return w;
}

controller_interface::return_type CartesianComplianceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    return controller_interface::return_type::OK;  // first cycle or a hiccup, hold
  }
  const Eigen::VectorXd q = readJoints();
  const Vector6d raw = readSensor();
  if (!q.allFinite() || !raw.allFinite()) {
    return controller_interface::return_type::OK;
  }

  // zero the sensor while holding still
  if (tare_count_ < tare_samples_) {
    bias_sum_ += raw;
    if (++tare_count_ == tare_samples_) {
      bias_ = bias_sum_ / tare_samples_;
    }
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      (void)command_interfaces_[i].set_value(q_cmd_[static_cast<Eigen::Index>(i)]);
    }
    publishState(time, q);
    return controller_interface::return_type::OK;
  }

  // pick up a new target from the topic
  if (const auto msg = *target_buffer_.readFromRT()) {
    target_ = fromMsg(msg->pose);
  }
  reference_ = stepTowards(
    reference_, target_, reference_linear_velocity_ * dt, reference_angular_velocity_ * dt);

  // the measured wrench, low-passed, deadbanded and clamped
  const double alpha = 1.0 - std::exp(-2.0 * M_PI * filter_cutoff_hz_ * dt);
  wrench_filtered_ += alpha * (externalWrench(raw, q) - wrench_filtered_);
  const Vector6d wrench = clampWrench(
    deadband(wrench_filtered_, force_deadband_, torque_deadband_), max_force_, max_torque_);

  // Admittance to compliant pose to joint command. The IK starts from the previous command and
  // not from the measured joints, so the servo's tracking error under load is what produces the
  // contact force.
  const Eigen::Isometry3d & compliant = admittance_.step(reference_, wrench, dt);
  const Eigen::VectorXd q_next = tcp_chain_->ikStep(
    q_cmd_, compliant, ik_iterations_, ik_damping_, max_joint_velocity_ * dt);
  if (q_next.allFinite()) {
    q_cmd_ = q_next;
  }
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    (void)command_interfaces_[i].set_value(q_cmd_[static_cast<Eigen::Index>(i)]);
  }
  publishState(time, q);
  return controller_interface::return_type::OK;
}

void CartesianComplianceController::publishState(const rclcpp::Time & time, const Eigen::VectorXd & q)
{
  if (cycle_++ % static_cast<std::size_t>(publish_decimation_) != 0) {
    return;
  }
  if (pose_pub_ && pose_pub_->trylock()) {
    pose_pub_->msg_.header.stamp = time;
    pose_pub_->msg_.header.frame_id = base_frame_;
    toMsg(tcp_chain_->fk(q), pose_pub_->msg_.pose);
    pose_pub_->unlockAndPublish();
  }
  if (wrench_pub_ && wrench_pub_->trylock()) {
    auto & m = wrench_pub_->msg_;
    m.header.stamp = time;
    m.header.frame_id = base_frame_;
    const Vector6d & w = wrench_filtered_;
    m.wrench.force.x = w[0];
    m.wrench.force.y = w[1];
    m.wrench.force.z = w[2];
    m.wrench.torque.x = w[3];
    m.wrench.torque.y = w[4];
    m.wrench.torque.z = w[5];
    wrench_pub_->unlockAndPublish();
  }
}

}  // namespace cell_controllers

PLUGINLIB_EXPORT_CLASS(
  cell_controllers::CartesianComplianceController, controller_interface::ControllerInterface)
