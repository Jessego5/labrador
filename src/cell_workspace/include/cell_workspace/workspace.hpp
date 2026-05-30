// This is the model of the cell's fixed furniture, the trays with their named slots, the bins
// and the insertion fixtures, and it is where every named target such as "tray_1/C4" turns into
// a pose. It is plain C++ over Eigen and yaml-cpp with no ROS in it, so the slot arithmetic and
// the config checks can be unit tested in milliseconds; the workspace node and the MJCF
// alignment test both link against it. Every pose is in the workspace
// frame (`Workspace::frame`, normally "table", whose origin is the robot base on the table top).
#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace cell_workspace
{

// zero-based slot coordinates, row 0 is "A" and col 0 is "1"
struct SlotIndex
{
  int row = 0;
  int col = 0;
};

// {row = 2, col = 3} for a slot name such as "C4". Throws std::invalid_argument when the name is
// not one upper-case letter followed by a number of at least 1.
SlotIndex parseSlotName(const std::string & name);

// "C4" for (2, 3), the inverse of parseSlotName. Throws std::out_of_range for a row outside
// [0, 25] or a negative column.
std::string slotName(int row, int col);

// an axis-aligned rectangle in the workspace XY plane
struct Footprint
{
  Eigen::Vector2d min = Eigen::Vector2d::Zero();
  Eigen::Vector2d max = Eigen::Vector2d::Zero();

  // True when the interiors overlap by more than `tolerance`. Edges that only touch do not
  // count, since two trays set side by side are a valid layout.
  bool overlaps(const Footprint & other, double tolerance = 1e-9) const;
};

// A grid tray. The tray frame sits at the centre of slot A1 on the top surface, with rows
// advancing along the tray's +x and columns along its +y, so a slot pose is just the origin plus
// row and column pitch, rotated by the tray's yaw.
struct Tray
{
  std::string name;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();  // slot A1 centre, on the top surface
  double yaw = 0.0;                                  // rad, about workspace z
  int rows = 0;
  int cols = 0;
  Eigen::Vector2d pitch = Eigen::Vector2d::Zero();   // (row spacing, column spacing)
  double slot_size = 0.0;                            // square pocket width
  double slot_depth = 0.0;                           // pocket depth below the top surface

  // the tray frame, at slot A1
  Eigen::Isometry3d pose() const;

  // The centre of a slot's opening on the top surface, oriented like the tray. Throws
  // std::out_of_range when the slot is outside the grid.
  Eigen::Isometry3d slotPose(int row, int col) const;
  Eigen::Isometry3d slotPose(const std::string & slot) const;

  // the outer footprint of the tray body, the grid plus its outer walls
  Footprint footprint() const;
};

// an open-top bin
struct Bin
{
  std::string name;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  // centre of the floor's top surface
  Eigen::Vector3d size = Eigen::Vector3d::Zero();    // outer x, y and wall height

  Eigen::Isometry3d pose() const;
  Footprint footprint() const;
};

// a block with a square insertion hole
struct Fixture
{
  std::string name;
  Eigen::Isometry3d hole_pose = Eigen::Isometry3d::Identity();  // centre of the hole opening
  double hole_diameter = 0.0;
  double insert_depth = 0.0;
  double block_size = 0.08;  // square footprint of the block

  Footprint footprint() const;
};

class Workspace
{
public:
  // a named pose to broadcast on TF
  struct NamedFrame
  {
    std::string target;    // e.g. "tray_1/C4"
    std::string frame_id;  // e.g. "tray_1_C4"
    Eigen::Isometry3d pose;
  };

  std::string frame = "table";
  std::vector<Tray> trays;
  std::vector<Bin> bins;
  std::vector<Fixture> fixtures;

  // One human-readable message per problem in the configuration, empty when it is valid. Every
  // problem is reported at once rather than stopping at the first, so a bad edit to
  // workspace.yaml is fixed in one pass. Overlap is checked between every pair of footprints
  // because an overlap is easy to make and invisible until the robot hits something: the first
  // layout of this cell put the peg fixture on top of tray_1's row D.
  std::vector<std::string> validate() const;

  // The pose of "tray_1", "tray_1/C4", "bin_a" or "peg_fixture". Throws std::invalid_argument
  // for an unknown name and std::out_of_range for a slot that is not there.
  Eigen::Isometry3d resolve(const std::string & target) const;

  // every tray, slot, bin and fixture frame, for TF
  std::vector<NamedFrame> allFrames() const;

  // the TF frame id for a target, with '/' replaced by '_' ("tray_1/C4" becomes "tray_1_C4")
  static std::string frameId(const std::string & target);
};

// Load and parse a workspace YAML file or string. This does not validate, call
// Workspace::validate() for that; it throws std::runtime_error naming the offending key path
// when the YAML itself is malformed.
Workspace loadWorkspaceFile(const std::string & path);
Workspace loadWorkspaceString(const std::string & yaml);

}  // namespace cell_workspace
