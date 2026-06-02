// This is the implementation of the top-down grasp geometry declared in grasp_geometry.hpp: the
// yaw folding, the downward TCP pose and the grasp height rule. It has no ROS in it, so
// test_grasp_geometry.cpp checks it directly.
#include "cell_skills/grasp_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cell_skills
{

double yawOf(const Eigen::Matrix3d & rotation)
{
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

double foldYaw(double yaw)
{
  constexpr double quarter = M_PI / 2.0;
  double folded = std::remainder(yaw, quarter);  // now in [-pi/4, pi/4]
  if (folded <= -M_PI / 4.0 + 1e-12) {
    folded += quarter;  // the two ends are the same grasp, prefer +pi/4 over -pi/4
  }
  return folded;
}

Eigen::Isometry3d topDownPose(const Eigen::Vector3d & position, double yaw)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = position;
  pose.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
  return pose;
}

double graspHeight(double bottom, double top, const GraspParams & params)
{
  const double lowest = bottom + params.fingertip_reach + params.surface_clearance;
  const double height = std::max(top - params.grasp_depth, lowest);
  if (height >= top) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return height;
}

}  // namespace cell_skills
