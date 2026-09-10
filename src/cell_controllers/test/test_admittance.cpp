// These are the tests for the admittance math and the KDL kinematics in admittance.hpp. They pin
// the pose algebra, the wrench clamp and deadband, the steady-state deflection of F/K, no
// overshoot when critically damped, tracking a moving reference without lag (the reason damping
// acts on the relative velocity), the velocity and workspace clamps, and the FK, Jacobian and IK
// step on a small planar arm, so none of it needs a robot or a simulator. Run them with colcon
// test --packages-select cell_controllers.
#include <cmath>

#include <gtest/gtest.h>
#include <kdl/chain.hpp>

#include "cell_controllers/admittance.hpp"

using cell_controllers::Admittance;
using cell_controllers::AdmittanceParams;
using cell_controllers::ChainKinematics;
using cell_controllers::Vector6d;

namespace
{

Eigen::Isometry3d pose(double x, double y, double z, double yaw = 0.0)
{
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(x, y, z);
  p.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return p;
}

AdmittanceParams params()
{
  AdmittanceParams p;
  p.mass = Vector6d::Constant(2.0);
  p.stiffness << 400, 400, 400, 30, 30, 30;
  p.damping = AdmittanceParams::criticalDamping(p.mass, p.stiffness, 1.0);
  p.max_linear_velocity = 1.0;
  p.max_angular_velocity = 5.0;
  return p;
}

// a planar 3R arm in the x-y plane, the last link a short yaw-only wrist, as a KDL chain for the
// kinematics tests
KDL::Chain planarArm()
{
  KDL::Chain c;
  c.addSegment(KDL::Segment("l1", KDL::Joint("j1", KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.4, 0, 0))));
  c.addSegment(KDL::Segment("l2", KDL::Joint("j2", KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.3, 0, 0))));
  c.addSegment(KDL::Segment("l3", KDL::Joint("j3", KDL::Joint::RotZ), KDL::Frame(KDL::Vector(0.1, 0, 0))));
  return c;
}

}  // namespace

TEST(PoseError, TranslationAndRotation)
{
  const Vector6d e = cell_controllers::poseError(pose(1, 2, 3, 0.3), pose(0.5, 2, 3, 0.1));
  EXPECT_NEAR(e[0], 0.5, 1e-12);
  EXPECT_NEAR(e[1], 0.0, 1e-12);
  EXPECT_NEAR(e[5], 0.2, 1e-12);
  EXPECT_NEAR(e[3], 0.0, 1e-12);
}

TEST(Integrate, InvertsPoseError)
{
  const auto a = pose(0.1, -0.2, 0.3, 0.4);
  const auto b = pose(0.0, 0.0, 0.25, -0.2);
  const auto c = cell_controllers::integrate(b, cell_controllers::poseError(a, b), 1.0);
  EXPECT_LT(cell_controllers::poseError(a, c).norm(), 1e-9);
}

TEST(StepTowards, LimitsTheStep)
{
  const auto s = cell_controllers::stepTowards(pose(0, 0, 0), pose(1, 0, 0, 1.0), 0.01, 0.02);
  EXPECT_NEAR(s.translation().x(), 0.01, 1e-12);
  EXPECT_NEAR(cell_controllers::poseError(s, pose(0, 0, 0))[5], 0.02, 1e-12);
  const auto t = cell_controllers::stepTowards(pose(0, 0, 0), pose(0.001, 0, 0), 0.01, 0.02);
  EXPECT_NEAR(t.translation().x(), 0.001, 1e-12);
}

TEST(ClampWrench, ScalesForceAndTorqueSeparately)
{
  Vector6d w;
  w << 30, 40, 0, 0, 0, 1;
  const Vector6d c = cell_controllers::clampWrench(w, 10.0, 5.0);
  EXPECT_NEAR(c.head<3>().norm(), 10.0, 1e-12);
  EXPECT_NEAR(c[0] / c[1], 0.75, 1e-12);
  EXPECT_NEAR(c[5], 1.0, 1e-12);
}

TEST(Deadband, ZeroesSmallAndShrinksLarge)
{
  Vector6d w;
  w << 0.2, -1.0, 5.0, 0.01, -0.5, 0.0;
  const Vector6d d = cell_controllers::deadband(w, 0.5, 0.05);
  EXPECT_DOUBLE_EQ(d[0], 0.0);
  EXPECT_DOUBLE_EQ(d[1], -0.5);
  EXPECT_DOUBLE_EQ(d[2], 4.5);
  EXPECT_DOUBLE_EQ(d[3], 0.0);
  EXPECT_DOUBLE_EQ(d[4], -0.45);
}

TEST(Admittance, ConvergesToReferenceWithoutContact)
{
  Admittance a(params());
  a.reset(pose(0, 0, 0));
  const auto ref = pose(0.02, -0.01, 0.03, 0.1);
  for (int i = 0; i < 2000; ++i) {
    a.step(ref, Vector6d::Zero(), 0.004);
  }
  EXPECT_LT(cell_controllers::poseError(a.pose(), ref).norm(), 1e-6);
}

TEST(Admittance, SteadyStateDeflectionIsForceOverStiffness)
{
  Admittance a(params());
  a.reset(pose(0, 0, 0));
  Vector6d f = Vector6d::Zero();
  f[2] = 8.0;   // the environment pushes the tool up with 8 N
  f[5] = 0.6;   // and twists it
  for (int i = 0; i < 3000; ++i) {
    a.step(pose(0, 0, 0), f, 0.004);
  }
  EXPECT_NEAR(a.pose().translation().z(), 8.0 / 400.0, 1e-6);
  EXPECT_NEAR(cell_controllers::poseError(a.pose(), pose(0, 0, 0))[5], 0.6 / 30.0, 1e-6);
}

TEST(Admittance, CriticallyDampedDoesNotOvershoot)
{
  // pushed away from a stationary reference and released, it settles back without crossing it
  Admittance a(params());
  a.reset(pose(0, 0, 0));
  Vector6d push = Vector6d::Zero();
  push[0] = -4.0;  // 4 N is 10 mm at 400 N/m
  for (int i = 0; i < 3000; ++i) {
    a.step(pose(0, 0, 0), push, 0.004);
  }
  ASSERT_NEAR(a.pose().translation().x(), -0.01, 1e-5);
  double max_x = -1.0;
  for (int i = 0; i < 3000; ++i) {
    max_x = std::max(max_x, a.step(pose(0, 0, 0), Vector6d::Zero(), 0.004).translation().x());
  }
  EXPECT_LT(max_x, 1e-5);
  EXPECT_GT(max_x, -1e-4);
}

TEST(Admittance, FollowsMovingReferenceWithoutLag)
{
  auto p = params();
  p.damping = Vector6d::Constant(400.0);  // heavily overdamped
  Admittance a(p);
  a.reset(pose(0, 0, 0));
  const double dt = 0.004, v = 0.01;
  for (int i = 1; i <= 3000; ++i) {
    a.step(pose(v * dt * i, 0, 0), Vector6d::Zero(), dt);
  }
  // semi-implicit integration leads by one step (v * dt = 0.04 mm)
  EXPECT_NEAR(a.pose().translation().x(), v * dt * 3000, 1.5 * v * dt);
}

TEST(Admittance, VelocityAndWorkspaceClamps)
{
  auto p = params();
  p.max_linear_velocity = 0.02;
  p.workspace_min = Eigen::Vector3d(-1, -1, 0.05);
  Admittance a(p);
  a.reset(pose(0, 0, 0.1));
  const double dt = 0.004;
  double prev = 0.1;
  for (int i = 0; i < 2000; ++i) {
    const double z = a.step(pose(0, 0, -0.5), Vector6d::Zero(), dt).translation().z();
    EXPECT_LE(prev - z, 0.02 * dt + 1e-12);
    EXPECT_GE(z, 0.05 - 1e-12);
    prev = z;
  }
  EXPECT_NEAR(prev, 0.05, 1e-12);
}

TEST(ChainKinematics, FkAndJacobian)
{
  ChainKinematics k(planarArm());
  ASSERT_EQ(k.joints(), 3u);
  EXPECT_EQ(k.jointNames(), (std::vector<std::string>{"j1", "j2", "j3"}));
  Eigen::VectorXd q(3);
  q << 0.3, -0.5, 0.2;
  const auto x = k.fk(q);
  EXPECT_NEAR(x.translation().x(), 0.4 * std::cos(0.3) + 0.3 * std::cos(-0.2) + 0.1 * std::cos(0.0), 1e-12);
  // the Jacobian matches finite differences
  const Eigen::MatrixXd J = k.jacobian(q);
  for (int j = 0; j < 3; ++j) {
    Eigen::VectorXd dq = q;
    dq[j] += 1e-7;
    const Vector6d fd = cell_controllers::poseError(k.fk(dq), x) / 1e-7;
    EXPECT_LT((fd - J.col(j)).norm(), 1e-5);
  }
}

TEST(ChainKinematics, IkStepReachesNearbyTarget)
{
  ChainKinematics k(planarArm());
  Eigen::VectorXd q(3);
  q << 0.3, -0.5, 0.2;
  Eigen::VectorXd q_target(3);
  q_target << 0.32, -0.52, 0.25;
  const auto target = k.fk(q_target);
  Eigen::VectorXd qi = q;
  for (int i = 0; i < 20; ++i) {
    qi = k.ikStep(qi, target, 2, 1e-3, 0.01);
  }
  const Vector6d e = cell_controllers::poseError(target, k.fk(qi));
  EXPECT_LT(e.head<2>().norm(), 1e-6);
  EXPECT_LT(std::abs(e[5]), 1e-6);
}

TEST(ChainKinematics, IkStepRespectsMaxStep)
{
  ChainKinematics k(planarArm());
  Eigen::VectorXd q(3);
  q << 0.3, -0.5, 0.2;
  Eigen::VectorXd far(3);
  far << 1.0, 0.5, -0.5;
  const auto next = k.ikStep(q, k.fk(far), 2, 1e-3, 0.005);
  EXPECT_LE((next - q).cwiseAbs().maxCoeff(), 0.005 + 1e-12);
}

TEST(ChainKinematics, BadUrdfThrows)
{
  EXPECT_THROW(ChainKinematics("<robot/>", "a", "b"), std::runtime_error);
}
