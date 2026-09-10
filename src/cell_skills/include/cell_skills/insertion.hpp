// These are the search and monitoring helpers behind the Insert skill: the pattern of points a
// peck search probes, and the monitor that notices when an insertion has stopped making
// progress. They are plain C++ over Eigen with no ROS in them, so they are unit tested on their
// own in test_insertion.cpp, and the skill server only has to drive them.
#pragma once

#include <vector>

#include <Eigen/Core>

namespace cell_skills
{

// The probe offsets for a peck search: the points of a square grid with `spacing` that lie
// inside a disc of `radius`, starting with the centre and nearest first (ties broken by angle).
// With spacing s, every offset in the disc is within s/2 per axis of some probe, so a hole whose
// capture window is wider than +-s/2 cannot be missed. Throws std::invalid_argument when the
// spacing is not positive or the radius is negative.
//
// Probing (lift, move, press) rather than dragging the part across the surface matters for a
// long part held near its top. A spiral search that dragged the part was tried and failed: the
// held peg pivots between the gripper pads, which give it no torsional resistance, so its tip
// lagged the gripper by up to 6 mm.
std::vector<Eigen::Vector2d> peckSearchPattern(double spacing, double radius);

// Detects a stall, meaning `value` (the insertion depth, say) has not improved on its best by
// at least `min_progress` for `window` seconds.
class ProgressMonitor
{
public:
  ProgressMonitor(double min_progress, double window);

  void reset(double t, double value);
  // feed a sample, true when stalled
  bool update(double t, double value);
  double best() const {return best_;}

private:
  double min_progress_;
  double window_;
  double best_ = 0.0;
  double best_time_ = 0.0;
};

}  // namespace cell_skills
