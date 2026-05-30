// This is the implementation of the workspace model declared in workspace.hpp: slot names, the
// pose and footprint arithmetic for trays, bins and fixtures, the configuration checks and the
// lookup that turns a target name into a pose. Like the header it has no ROS in it, so
// test_workspace.cpp can exercise all of it directly.
#include "cell_workspace/workspace.hpp"

#include <algorithm>
#include <cctype>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace cell_workspace
{
namespace
{

Eigen::Isometry3d yawPose(const Eigen::Vector3d & position, double yaw)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = position;
  pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

// the axis-aligned bounds of the local rectangle [lo, hi] after transforming it by `pose`
Footprint transformedFootprint(
  const Eigen::Isometry3d & pose, const Eigen::Vector2d & lo, const Eigen::Vector2d & hi)
{
  constexpr double inf = std::numeric_limits<double>::infinity();
  Footprint fp{Eigen::Vector2d::Constant(inf), Eigen::Vector2d::Constant(-inf)};
  for (double x : {lo.x(), hi.x()}) {
    for (double y : {lo.y(), hi.y()}) {
      const Eigen::Vector2d corner = (pose * Eigen::Vector3d(x, y, 0.0)).head<2>();
      fp.min = fp.min.cwiseMin(corner);
      fp.max = fp.max.cwiseMax(corner);
    }
  }
  return fp;
}

std::string sizeString(int rows, int cols)
{
  return std::to_string(rows) + "x" + std::to_string(cols);
}

}  // namespace

// Slot names

SlotIndex parseSlotName(const std::string & name)
{
  const auto fail = [&name]() {
      return std::invalid_argument(
        "invalid slot name '" + name +
        "': expected a row letter A-Z followed by a column number >= 1, e.g. C4");
    };
  if (name.size() < 2 || name.size() > 4 || name[0] < 'A' || name[0] > 'Z') {
    throw fail();
  }
  const std::string digits = name.substr(1);
  const bool all_digits = std::all_of(
    digits.begin(), digits.end(), [](unsigned char c) {return std::isdigit(c) != 0;});
  if (!all_digits || digits[0] == '0') {
    throw fail();
  }
  return SlotIndex{name[0] - 'A', std::stoi(digits) - 1};
}

std::string slotName(int row, int col)
{
  if (row < 0 || row > 25 || col < 0) {
    throw std::out_of_range(
            "slot index (" + std::to_string(row) + ", " + std::to_string(col) + ") has no name");
  }
  return std::string(1, static_cast<char>('A' + row)) + std::to_string(col + 1);
}

// Geometry

bool Footprint::overlaps(const Footprint & other, double tolerance) const
{
  return min.x() < other.max.x() - tolerance && other.min.x() < max.x() - tolerance &&
         min.y() < other.max.y() - tolerance && other.min.y() < max.y() - tolerance;
}

Eigen::Isometry3d Tray::pose() const
{
  return yawPose(origin, yaw);
}

Eigen::Isometry3d Tray::slotPose(int row, int col) const
{
  if (row < 0 || row >= rows || col < 0 || col >= cols) {
    throw std::out_of_range(
            "slot (row " + std::to_string(row) + ", col " + std::to_string(col) +
            ") is outside tray '" + name + "' (" + sizeString(rows, cols) + ")");
  }
  Eigen::Isometry3d slot = pose();
  slot.translate(Eigen::Vector3d(row * pitch.x(), col * pitch.y(), 0.0));
  return slot;
}

Eigen::Isometry3d Tray::slotPose(const std::string & slot) const
{
  const SlotIndex index = parseSlotName(slot);
  if (index.row >= rows || index.col >= cols) {
    throw std::out_of_range(
            "slot '" + slot + "' is outside tray '" + name + "' (" + sizeString(rows, cols) +
            ": A1.." + slotName(rows - 1, cols - 1) + ")");
  }
  return slotPose(index.row, index.col);
}

Footprint Tray::footprint() const
{
  return transformedFootprint(
    pose(), Eigen::Vector2d(-0.5 * pitch.x(), -0.5 * pitch.y()),
    Eigen::Vector2d((rows - 0.5) * pitch.x(), (cols - 0.5) * pitch.y()));
}

Eigen::Isometry3d Bin::pose() const
{
  return yawPose(center, 0.0);
}

Footprint Bin::footprint() const
{
  const Eigen::Vector2d half = 0.5 * size.head<2>();
  return Footprint{center.head<2>() - half, center.head<2>() + half};
}

Footprint Fixture::footprint() const
{
  const Eigen::Vector2d half = Eigen::Vector2d::Constant(0.5 * block_size);
  return transformedFootprint(hole_pose, -half, half);
}

// Workspace

std::vector<std::string> Workspace::validate() const
{
  std::vector<std::string> errors;
  const auto error = [&errors](std::string message) {errors.push_back(std::move(message));};

  if (frame.empty()) {
    error("frame must not be empty");
  }

  std::set<std::string> names;
  const auto check_name = [&](const std::string & kind, const std::string & name) {
      if (name.empty()) {
        error(kind + " with an empty name");
      } else if (name.find('/') != std::string::npos) {
        error(kind + " '" + name + "': names must not contain '/'");
      } else if (!names.insert(name).second) {
        error("duplicate name '" + name + "'");
      }
    };

  std::vector<std::pair<std::string, Footprint>> footprints;

  for (const auto & t : trays) {
    const std::string where = "tray '" + t.name + "'";
    check_name("tray", t.name);
    if (t.rows < 1 || t.rows > 26) {
      error(where + ": rows must be 1..26 (got " + std::to_string(t.rows) + ")");
    }
    if (t.cols < 1 || t.cols > 999) {
      error(where + ": cols must be 1..999 (got " + std::to_string(t.cols) + ")");
    }
    if (t.pitch.x() <= 0.0 || t.pitch.y() <= 0.0) {
      error(where + ": pitch must be positive");
    }
    if (t.slot_size <= 0.0) {
      error(where + ": slot_size must be positive");
    } else if (t.slot_size >= t.pitch.minCoeff()) {
      error(where + ": slots overlap (slot_size " + std::to_string(t.slot_size) +
        " >= pitch " + std::to_string(t.pitch.minCoeff()) + ")");
    }
    if (t.slot_depth <= 0.0) {
      error(where + ": slot_depth must be positive");
    } else if (t.slot_depth >= t.origin.z()) {
      error(where + ": slot_depth must be less than the tray height (origin z)");
    }
    footprints.emplace_back(where, t.footprint());
  }

  for (const auto & b : bins) {
    const std::string where = "bin '" + b.name + "'";
    check_name("bin", b.name);
    if ((b.size.array() <= 0.0).any()) {
      error(where + ": size must be positive");
    } else if (b.size.z() <= b.center.z()) {
      error(where + ": wall height (size z) must exceed the floor height (center z)");
    }
    if (b.center.z() < 0.0) {
      error(where + ": floor (center z) must not be below the table");
    }
    footprints.emplace_back(where, b.footprint());
  }

  for (const auto & f : fixtures) {
    const std::string where = "fixture '" + f.name + "'";
    check_name("fixture", f.name);
    if (f.hole_diameter <= 0.0 || f.insert_depth <= 0.0) {
      error(where + ": hole_diameter and insert_depth must be positive");
    }
    if (f.block_size <= f.hole_diameter) {
      error(where + ": block_size must exceed hole_diameter");
    }
    if (f.insert_depth >= f.hole_pose.translation().z()) {
      error(where + ": insert_depth must be less than the hole height (hole z)");
    }
    footprints.emplace_back(where, f.footprint());
  }

  for (std::size_t i = 0; i < footprints.size(); ++i) {
    for (std::size_t j = i + 1; j < footprints.size(); ++j) {
      if (footprints[i].second.overlaps(footprints[j].second)) {
        error(footprints[i].first + " overlaps " + footprints[j].first);
      }
    }
  }
  return errors;
}

Eigen::Isometry3d Workspace::resolve(const std::string & target) const
{
  const auto slash = target.find('/');
  const std::string base = target.substr(0, slash);

  for (const auto & t : trays) {
    if (t.name == base) {
      return slash == std::string::npos ? t.pose() : t.slotPose(target.substr(slash + 1));
    }
  }
  if (slash == std::string::npos) {
    for (const auto & b : bins) {
      if (b.name == target) {
        return b.pose();
      }
    }
    for (const auto & f : fixtures) {
      if (f.name == target) {
        return f.hole_pose;
      }
    }
  }
  throw std::invalid_argument("unknown workspace target '" + target + "'");
}

std::vector<Workspace::NamedFrame> Workspace::allFrames() const
{
  std::vector<NamedFrame> frames;
  for (const auto & t : trays) {
    frames.push_back({t.name, frameId(t.name), t.pose()});
    for (int r = 0; r < t.rows; ++r) {
      for (int c = 0; c < t.cols; ++c) {
        const std::string target = t.name + "/" + slotName(r, c);
        frames.push_back({target, frameId(target), t.slotPose(r, c)});
      }
    }
  }
  for (const auto & b : bins) {
    frames.push_back({b.name, frameId(b.name), b.pose()});
  }
  for (const auto & f : fixtures) {
    frames.push_back({f.name, frameId(f.name), f.hole_pose});
  }
  return frames;
}

std::string Workspace::frameId(const std::string & target)
{
  std::string id = target;
  std::replace(id.begin(), id.end(), '/', '_');
  return id;
}

}  // namespace cell_workspace
