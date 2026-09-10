// This is the math behind the compliance controller: the Cartesian admittance dynamics, the small
// pose and wrench helpers it is built from, and the KDL kinematics that turn a compliant pose into
// joint positions. There is no ROS here, only Eigen and KDL, so test_admittance can check the
// dynamics and the IK step in isolation; CartesianComplianceController is the only thing that
// uses it on the robot. Poses and wrenches are all in the common base frame.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace KDL
{
class Chain;
class ChainFkSolverPos_recursive;
class ChainJntToJacSolver;
}  // namespace KDL

namespace cell_controllers
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// The twist-like difference a ⊖ b: the position difference stacked on the rotation vector of
// R_a R_b^T, both in the common (base) frame.
Vector6d poseError(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b);

// `pose` moved by `twist * dt`, with the linear part added and the angular part applied in the
// base frame.
Eigen::Isometry3d integrate(const Eigen::Isometry3d & pose, const Vector6d & twist, double dt);

// `from` moved towards `to` by at most `max_linear` metres and `max_angular` radians.
Eigen::Isometry3d stepTowards(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & to, double max_linear,
  double max_angular);

// The wrench with its force and torque parts scaled separately, so neither exceeds its norm
// limit.
Vector6d clampWrench(const Vector6d & wrench, double max_force, double max_torque);

// The wrench with each component below the deadband zeroed and the others shrunk by it, so
// contact noise does not make the compliant frame drift.
Vector6d deadband(const Vector6d & wrench, double force_band, double torque_band);

struct AdmittanceParams
{
  Vector6d mass = Vector6d::Constant(1.0);       // kg (linear), kg m^2 (angular)
  Vector6d stiffness = Vector6d::Constant(100);  // N/m, N m/rad
  Vector6d damping = Vector6d::Constant(20);     // N s/m, N m s/rad
  double max_linear_velocity = 0.05;             // m/s of the compliant frame
  double max_angular_velocity = 0.3;             // rad/s
  Eigen::Vector3d workspace_min = Eigen::Vector3d::Constant(-10);  // position bounds
  Eigen::Vector3d workspace_max = Eigen::Vector3d::Constant(10);

  // the damping for a given damping ratio, d = 2 zeta sqrt(k m) per axis
  static Vector6d criticalDamping(const Vector6d & mass, const Vector6d & stiffness, double zeta);
};

// A virtual mass-spring-damper between a reference pose and the commanded ("compliant") pose:
//
//   M (a - a_ref) + D (v - v_ref) + K (x_c ⊖ x_ref) = F_ext      (a_ref neglected)
//
// With no contact the compliant frame settles on the reference, and follows a moving reference
// without lag because the damping acts on the velocity relative to it. In contact it settles
// where the spring balances the external wrench, so the steady contact force is K times how far
// the reference has been pushed past the surface. D is sized for stable contact with a stiff
// environment, which leaves free-space motion overdamped.
class Admittance
{
public:
  explicit Admittance(AdmittanceParams params = {});

  void reset(const Eigen::Isometry3d & pose);
  // The new compliant pose after advancing by dt under the external wrench, which is the wrench
  // the environment applies to the tool at the controlled point, in the base frame.
  const Eigen::Isometry3d & step(
    const Eigen::Isometry3d & reference, const Vector6d & external_wrench, double dt);

  const Eigen::Isometry3d & pose() const {return pose_;}
  const Vector6d & velocity() const {return velocity_;}
  const AdmittanceParams & params() const {return params_;}
  void setParams(const AdmittanceParams & params) {params_ = params;}

private:
  AdmittanceParams params_;
  Eigen::Isometry3d pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d previous_reference_ = Eigen::Isometry3d::Identity();
  Vector6d velocity_ = Vector6d::Zero();
};

// forward kinematics, the Jacobian and a damped-least-squares IK step for a serial chain, by KDL
class ChainKinematics
{
public:
  // The chain from `base` to `tip` of the robot in `urdf`. Throws std::runtime_error when the
  // URDF does not parse or has no such chain.
  ChainKinematics(const std::string & urdf, const std::string & base, const std::string & tip);
  explicit ChainKinematics(const KDL::Chain & chain);
  ~ChainKinematics();
  ChainKinematics(const ChainKinematics &) = delete;
  ChainKinematics & operator=(const ChainKinematics &) = delete;

  std::size_t joints() const;
  std::vector<std::string> jointNames() const;

  Eigen::Isometry3d fk(const Eigen::VectorXd & q) const;
  // the 6 x n geometric Jacobian in the base frame, with the reference point at the tip
  Eigen::MatrixXd jacobian(const Eigen::VectorXd & q) const;

  // Joint positions near `q` that move the tip towards `target`, after `iterations` damped
  // least-squares steps, with each joint changing by at most `max_step` in total.
  Eigen::VectorXd ikStep(
    const Eigen::VectorXd & q, const Eigen::Isometry3d & target, int iterations, double damping,
    double max_step) const;

private:
  std::unique_ptr<KDL::Chain> chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_;
};

}  // namespace cell_controllers
