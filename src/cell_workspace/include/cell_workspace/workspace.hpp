// This is the model of the cell's fixed furniture, the trays with their named slots, the bins
// and the insertion fixtures, and it is where every named target such as "tray_1/C4" turns into
// a pose. It is plain C++ over Eigen and yaml-cpp with no ROS in it, so the slot arithmetic and
// the config checks can be unit tested in milliseconds; the workspace node, the skills, the
// executor and the MJCF alignment test all link against it. Every pose is in the workspace
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

// An open-top bin. Its floor can be split into a grid of drop spots, "bin_a/1" up to
// "bin_a/<rows*cols>", numbered row-major with rows along the bin's x and columns along its y.
// The spots exist because a bin is not a point: placing two cubes at the floor centre put the
// second one on top of the first.
struct Bin
{
  std::string name;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  // centre of the floor's top surface
  Eigen::Vector3d size = Eigen::Vector3d::Zero();    // outer x, y and wall height
  double wall = 0.005;                               // wall thickness, the same as the MJCF's
  int spot_rows = 1;                                 // drop spots along x
  int spot_cols = 1;                                 // drop spots along y

  Eigen::Isometry3d pose() const;
  Footprint footprint() const;

  int spotCount() const {return spot_rows * spot_cols;}

  // The centre of drop spot `n` (1-based) on the floor, which is the centre of the n-th cell of
  // an even grid over the inner floor, inside the walls. Throws std::out_of_range outside
  // 1..spotCount().
  Eigen::Isometry3d spotPose(int n) const;
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

  // The pose of "tray_1", "tray_1/C4", "bin_a", "bin_a/2" (a drop spot) or "peg_fixture".
  // Throws std::invalid_argument for an unknown name and std::out_of_range for a slot or spot
  // that is not there.
  Eigen::Isometry3d resolve(const std::string & target) const;

  // Where a placed object's bottom centre should end up, oriented like the target: the pocket
  // floor for "tray_1/C4", the floor for "bin_a" and the bottom of the hole for "peg_fixture".
  // This is not the same as resolve(), which gives the opening a robot aims at; a place has to
  // know how far down the object comes to rest. A tray with a single slot can be named on its
  // own, which places in that slot. Throws std::invalid_argument for an unknown name or the bare
  // name of a tray with several slots ("tray_1"), and std::out_of_range for a bad slot.
  Eigen::Isometry3d placePose(const std::string & target) const;

  // the bin called `name`, or nullptr when there is none
  const Bin * findBin(const std::string & name) const;

  // Every tray, slot, bin and fixture frame, for TF. Bin drop spots are left out: they are
  // derived from the bin and have no MuJoCo site for the alignment test to check them against.
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
