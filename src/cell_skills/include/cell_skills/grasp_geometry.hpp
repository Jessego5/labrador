// This is the geometry of a top-down grasp: which way the gripper should face an object and
// how high the TCP should hold it. Pick and Place in the skill server use it to turn an object
// pose and size into TCP poses. It is plain C++ over Eigen with no ROS in it, so the rules are
// unit tested on their own in test_grasp_geometry.cpp.
#pragma once

#include <Eigen/Geometry>

namespace cell_skills
{

// the gripper dimensions and margins that decide how high to hold an object
struct GraspParams
{
  double grasp_depth = 0.015;        // hold this far below the object's top
  double fingertip_reach = 0.018;    // how far the closed finger pads extend past the TCP
  double surface_clearance = 0.003;  // keep the fingertips this far above the support surface
};

// the yaw of a frame's x axis about world z
double yawOf(const Eigen::Matrix3d & rotation);

// The yaw folded into [-pi/4, pi/4]. A parallel gripper is symmetric under a half turn and boxes
// under a quarter turn, so this picks the equivalent grasp that needs the least wrist rotation.
double foldYaw(double yaw);

// a TCP pose pointing straight down (TCP z = -world z), rotated by `yaw` about world z
Eigen::Isometry3d topDownPose(const Eigen::Vector3d & position, double yaw);

// The TCP height for grasping an object whose bounding box spans [bottom, top] in z. It holds
// `grasp_depth` below the top, but never so low that the fingertips come within
// `surface_clearance` of the surface the object stands on. NaN when the object is too short to
// grasp at all, because the height would be at or above its top.
double graspHeight(double bottom, double top, const GraspParams & params);

}  // namespace cell_skills
