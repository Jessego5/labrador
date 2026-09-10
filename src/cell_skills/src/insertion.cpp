// This is the implementation of the insertion helpers declared in insertion.hpp, the peck
// search pattern and the progress monitor. It has no ROS in it, so test_insertion.cpp checks it
// directly.
#include "cell_skills/insertion.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cell_skills
{

std::vector<Eigen::Vector2d> peckSearchPattern(double spacing, double radius)
{
  if (spacing <= 0.0 || radius < 0.0) {
    throw std::invalid_argument("peckSearchPattern: spacing must be positive, radius >= 0");
  }
  // walk the square grid that bounds the disc and keep the points inside it
  const int n = static_cast<int>(std::floor(radius / spacing));
  std::vector<Eigen::Vector2d> points;
  for (int i = -n; i <= n; ++i) {
    for (int j = -n; j <= n; ++j) {
      const Eigen::Vector2d p(i * spacing, j * spacing);
      if (p.norm() <= radius + 1e-12) {
        points.push_back(p);
      }
    }
  }
  // nearest first, so the probes work outwards from the expected hole position
  std::stable_sort(
    points.begin(), points.end(), [](const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
      const double ra = a.norm(), rb = b.norm();
      if (std::abs(ra - rb) > 1e-9) {
        return ra < rb;
      }
      return std::atan2(a.y(), a.x()) < std::atan2(b.y(), b.x());
    });
  return points;
}

ProgressMonitor::ProgressMonitor(double min_progress, double window)
: min_progress_(min_progress), window_(window) {}

void ProgressMonitor::reset(double t, double value)
{
  best_ = value;
  best_time_ = t;
}

bool ProgressMonitor::update(double t, double value)
{
  // only real progress restarts the timer, small jitter around the best does not
  if (value >= best_ + min_progress_) {
    best_ = value;
    best_time_ = t;
    return false;
  }
  return t - best_time_ > window_;
}

}  // namespace cell_skills
