// These are the tests for the top-down grasp geometry. They pin the yaw folding, including its
// boundary, the direction of the downward TCP pose, and the grasp height rule on objects the
// size of the cell's cylinders and cubes, so a change to the margins shows up here before it
// shows up as a missed pick. Run them with colcon test --packages-select cell_skills.
#include <cmath>

#include <gtest/gtest.h>

#include "cell_skills/grasp_geometry.hpp"

using cell_skills::foldYaw;
using cell_skills::graspHeight;
using cell_skills::GraspParams;
using cell_skills::topDownPose;
using cell_skills::yawOf;

TEST(GraspGeometry, FoldYawStaysWithinAQuarterTurn)
{
  EXPECT_NEAR(foldYaw(0.0), 0.0, 1e-12);
  EXPECT_NEAR(foldYaw(M_PI / 2), 0.0, 1e-12);
  EXPECT_NEAR(foldYaw(M_PI), 0.0, 1e-12);
  EXPECT_NEAR(foldYaw(0.3 + M_PI / 2), 0.3, 1e-12);
  EXPECT_NEAR(foldYaw(-0.3 - M_PI), -0.3, 1e-12);
  EXPECT_NEAR(foldYaw(-M_PI / 4), M_PI / 4, 1e-12);  // the boundary maps to +pi/4
  for (double y = -10.0; y < 10.0; y += 0.37) {
    const double f = foldYaw(y);
    EXPECT_GT(f, -M_PI / 4 - 1e-9);
    EXPECT_LE(f, M_PI / 4 + 1e-9);
    // the same grasp up to a quarter turn
    EXPECT_NEAR(std::remainder(y - f, M_PI / 2), 0.0, 1e-9);
  }
}

TEST(GraspGeometry, TopDownPosePointsTheToolDown)
{
  const auto pose = topDownPose({0.5, 0.1, 0.2}, 0.4);
  EXPECT_TRUE(pose.translation().isApprox(Eigen::Vector3d(0.5, 0.1, 0.2)));
  EXPECT_TRUE((pose.linear() * Eigen::Vector3d::UnitZ()).isApprox(-Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_NEAR(yawOf(pose.linear()), 0.4, 1e-12);
}

TEST(GraspGeometry, GraspHeightHoldsNearTheTopButClearsTheSurface)
{
  const GraspParams p;  // depth 15 mm, reach 18 mm, clearance 3 mm
  // a 50 mm cylinder on the table is held 15 mm below its top
  EXPECT_NEAR(graspHeight(0.0, 0.05, p), 0.035, 1e-12);
  // a 40 mm cube is held at 25 mm, still at least 21 mm above the table
  EXPECT_NEAR(graspHeight(0.0, 0.04, p), 0.025, 1e-12);
  // a 30 mm block is pushed up to 21 mm so the fingertips clear the table
  EXPECT_NEAR(graspHeight(0.0, 0.03, p), 0.021, 1e-12);
  // a 20 mm block would have to be grasped above its top
  EXPECT_TRUE(std::isnan(graspHeight(0.0, 0.02, p)));
  // the same rule on a raised surface
  EXPECT_NEAR(graspHeight(0.10, 0.15, p), 0.135, 1e-12);
}
