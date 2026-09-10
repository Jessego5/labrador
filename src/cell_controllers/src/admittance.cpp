// This implements the admittance dynamics and the KDL kinematics declared in admittance.hpp. The
// admittance step integrates semi-implicitly (velocity first, then the pose with the new
// velocity) and applies two of the controller's safety limits on every step, the velocity limits
// on the compliant frame and the workspace box. The IK step is a plain damped least-squares
// update, J^T (J J^T + lambda^2 I)^-1 e, repeated a few times from the given joints, with the
// total change clamped per joint.
#include "cell_controllers/admittance.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace cell_controllers
{

Vector6d poseError(const Eigen::Isometry3d & a, const Eigen::Isometry3d & b)
{
  Vector6d e;
  e.head<3>() = a.translation() - b.translation();
  const Eigen::AngleAxisd aa(a.linear() * b.linear().transpose());
  e.tail<3>() = aa.angle() * aa.axis();
  return e;
}

Eigen::Isometry3d integrate(const Eigen::Isometry3d & pose, const Vector6d & twist, double dt)
{
  Eigen::Isometry3d out = pose;
  out.translation() += twist.head<3>() * dt;
  const Eigen::Vector3d w = twist.tail<3>() * dt;
  const double angle = w.norm();
  if (angle > 1e-12) {
    out.linear() = Eigen::AngleAxisd(angle, w / angle).toRotationMatrix() * pose.linear();
  }
  return out;
}

Eigen::Isometry3d stepTowards(
  const Eigen::Isometry3d & from, const Eigen::Isometry3d & to, double max_linear,
  double max_angular)
{
  const Vector6d e = poseError(to, from);
  const double lin = e.head<3>().norm();
  const double ang = e.tail<3>().norm();
  const double s_lin = lin > max_linear ? max_linear / lin : 1.0;
  const double s_ang = ang > max_angular ? max_angular / ang : 1.0;
  Vector6d step;
  step << s_lin * e.head<3>(), s_ang * e.tail<3>();
  return integrate(from, step, 1.0);
}

Vector6d clampWrench(const Vector6d & wrench, double max_force, double max_torque)
{
  Vector6d out = wrench;
  const double f = out.head<3>().norm();
  const double t = out.tail<3>().norm();
  if (f > max_force) {
    out.head<3>() *= max_force / f;
  }
  if (t > max_torque) {
    out.tail<3>() *= max_torque / t;
  }
  return out;
}

Vector6d deadband(const Vector6d & wrench, double force_band, double torque_band)
{
  Vector6d out;
  for (int i = 0; i < 6; ++i) {
    const double band = i < 3 ? force_band : torque_band;
    const double v = wrench[i];
    out[i] = std::abs(v) <= band ? 0.0 : v - std::copysign(band, v);
  }
  return out;
}

Vector6d AdmittanceParams::criticalDamping(
  const Vector6d & mass, const Vector6d & stiffness, double zeta)
{
  return 2.0 * zeta * (stiffness.cwiseProduct(mass)).cwiseSqrt();
}

Admittance::Admittance(AdmittanceParams params)
: params_(std::move(params)) {}

void Admittance::reset(const Eigen::Isometry3d & pose)
{
  pose_ = pose;
  previous_reference_ = pose;
  velocity_.setZero();
}

const Eigen::Isometry3d & Admittance::step(
  const Eigen::Isometry3d & reference, const Vector6d & external_wrench, double dt)
{
  const Vector6d e = poseError(pose_, reference);
  const Vector6d reference_velocity = poseError(reference, previous_reference_) / dt;
  previous_reference_ = reference;
  const Vector6d accel =
    (external_wrench - params_.stiffness.cwiseProduct(e) -
    params_.damping.cwiseProduct(velocity_ - reference_velocity))
    .cwiseQuotient(params_.mass);
  velocity_ += accel * dt;

  const double lin = velocity_.head<3>().norm();
  if (lin > params_.max_linear_velocity) {
    velocity_.head<3>() *= params_.max_linear_velocity / lin;
  }
  const double ang = velocity_.tail<3>().norm();
  if (ang > params_.max_angular_velocity) {
    velocity_.tail<3>() *= params_.max_angular_velocity / ang;
  }

  pose_ = integrate(pose_, velocity_, dt);

  // stop at a workspace bound and drop the velocity into it
  for (int i = 0; i < 3; ++i) {
    double & p = pose_.translation()[i];
    if (p < params_.workspace_min[i]) {
      p = params_.workspace_min[i];
      velocity_[i] = std::max(0.0, velocity_[i]);
    } else if (p > params_.workspace_max[i]) {
      p = params_.workspace_max[i];
      velocity_[i] = std::min(0.0, velocity_[i]);
    }
  }
  return pose_;
}

// Kinematics

namespace
{

Eigen::Isometry3d toEigen(const KDL::Frame & f)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  for (int r = 0; r < 3; ++r) {
    out.translation()[r] = f.p[r];
    for (int c = 0; c < 3; ++c) {
      out.linear()(r, c) = f.M(r, c);
    }
  }
  return out;
}

KDL::JntArray toKdl(const Eigen::VectorXd & q)
{
  KDL::JntArray out(static_cast<unsigned int>(q.size()));
  out.data = q;
  return out;
}

KDL::Chain chainFromUrdf(const std::string & urdf, const std::string & base, const std::string & tip)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf, tree)) {
    throw std::runtime_error("cannot parse the robot description into a KDL tree");
  }
  KDL::Chain chain;
  if (!tree.getChain(base, tip, chain)) {
    throw std::runtime_error("no kinematic chain from '" + base + "' to '" + tip + "'");
  }
  return chain;
}

}  // namespace

ChainKinematics::ChainKinematics(
  const std::string & urdf, const std::string & base, const std::string & tip)
: ChainKinematics(chainFromUrdf(urdf, base, tip)) {}

ChainKinematics::ChainKinematics(const KDL::Chain & chain)
: chain_(std::make_unique<KDL::Chain>(chain)),
  fk_(std::make_unique<KDL::ChainFkSolverPos_recursive>(*chain_)),
  jac_(std::make_unique<KDL::ChainJntToJacSolver>(*chain_)) {}

ChainKinematics::~ChainKinematics() = default;

std::size_t ChainKinematics::joints() const {return chain_->getNrOfJoints();}

std::vector<std::string> ChainKinematics::jointNames() const
{
  std::vector<std::string> names;
  for (const auto & segment : chain_->segments) {
    if (segment.getJoint().getType() != KDL::Joint::Fixed) {
      names.push_back(segment.getJoint().getName());
    }
  }
  return names;
}

Eigen::Isometry3d ChainKinematics::fk(const Eigen::VectorXd & q) const
{
  KDL::Frame f;
  if (fk_->JntToCart(toKdl(q), f) < 0) {
    throw std::runtime_error("forward kinematics failed");
  }
  return toEigen(f);
}

Eigen::MatrixXd ChainKinematics::jacobian(const Eigen::VectorXd & q) const
{
  KDL::Jacobian j(chain_->getNrOfJoints());
  if (jac_->JntToJac(toKdl(q), j) < 0) {
    throw std::runtime_error("Jacobian failed");
  }
  return j.data;
}

Eigen::VectorXd ChainKinematics::ikStep(
  const Eigen::VectorXd & q, const Eigen::Isometry3d & target, int iterations, double damping,
  double max_step) const
{
  Eigen::VectorXd out = q;
  const double lambda2 = damping * damping;
  for (int it = 0; it < iterations; ++it) {
    const Vector6d e = poseError(target, fk(out));
    const Eigen::MatrixXd J = jacobian(out);
    const Matrix6d A = J * J.transpose() + lambda2 * Matrix6d::Identity();
    out += J.transpose() * A.ldlt().solve(e);
  }
  const Eigen::VectorXd dq = (out - q).cwiseMax(-max_step).cwiseMin(max_step);
  return q + dq;
}

}  // namespace cell_controllers
