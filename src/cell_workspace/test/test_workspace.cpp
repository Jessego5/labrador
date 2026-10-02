// These are the tests for the workspace model, the ROS-free core in workspace.hpp. They pin the
// slot naming and slot arithmetic exactly (C4 is row 2, column 3, rows run along the tray's x),
// the pose every kind of target resolves to, where a placed object comes to rest, the error
// messages a malformed workspace.yaml produces, and the checks validate() makes, including the
// overlap check that catches a fixture sitting on a tray. They use a small inline workspace
// rather than the real workspace.yaml, so a layout change in the cell does not break them. Run
// them with colcon test --packages-select cell_workspace.
#include <cmath>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "cell_workspace/workspace.hpp"

using cell_workspace::Bin;
using cell_workspace::Fixture;
using cell_workspace::loadWorkspaceString;
using cell_workspace::parseSlotName;
using cell_workspace::slotName;
using cell_workspace::Tray;
using cell_workspace::Workspace;

namespace
{

constexpr double kTol = 1e-12;

const char * kValidYaml = R"(
frame: table
trays:
  tray_1:
    origin: [0.45, -0.10, 0.02]
    yaw: 0.0
    rows: 4
    cols: 6
    pitch: [0.040, 0.040]
    slot_size: 0.034
    slot_depth: 0.015
bins:
  bin_a: {center: [0.30, 0.30, 0.02], size: [0.15, 0.12, 0.08], spots: [2, 1]}
fixtures:
  peg_fixture:
    hole_pose: [0.66, 0.05, 0.05, 0, 0, 1.5707963267948966]
    hole_diameter: 0.022
    insert_depth: 0.030
)";

Tray makeTray(double yaw = 0.0)
{
  Tray t;
  t.name = "tray_1";
  t.origin = {0.45, -0.10, 0.02};
  t.yaw = yaw;
  t.rows = 4;
  t.cols = 6;
  t.pitch = {0.04, 0.05};
  t.slot_size = 0.034;
  t.slot_depth = 0.015;
  return t;
}

bool contains(const std::vector<std::string> & errors, const std::string & needle)
{
  for (const auto & e : errors) {
    if (e.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

// - slot names

TEST(SlotName, ParsesRowLetterAndColumnNumber)
{
  const auto a1 = parseSlotName("A1");
  EXPECT_EQ(a1.row, 0);
  EXPECT_EQ(a1.col, 0);
  const auto c4 = parseSlotName("C4");
  EXPECT_EQ(c4.row, 2);
  EXPECT_EQ(c4.col, 3);
  const auto z12 = parseSlotName("Z12");
  EXPECT_EQ(z12.row, 25);
  EXPECT_EQ(z12.col, 11);
}

TEST(SlotName, RejectsMalformedNames)
{
  for (const std::string bad : {"", "A", "a1", "1A", "A0", "A01", "AA1", "A1x", "A-1", "A 1",
      "A1234"})
  {
    EXPECT_THROW(parseSlotName(bad), std::invalid_argument) << "'" << bad << "'";
  }
}

TEST(SlotName, RoundTrips)
{
  for (int r = 0; r < 26; ++r) {
    for (int c = 0; c < 12; ++c) {
      const auto idx = parseSlotName(slotName(r, c));
      EXPECT_EQ(idx.row, r);
      EXPECT_EQ(idx.col, c);
    }
  }
  EXPECT_THROW(slotName(26, 0), std::out_of_range);
  EXPECT_THROW(slotName(0, -1), std::out_of_range);
}

// - slot poses

TEST(TrayMath, A1IsTheOrigin)
{
  const Tray t = makeTray(0.3);
  EXPECT_TRUE(t.slotPose("A1").isApprox(t.pose(), kTol));
  EXPECT_TRUE(t.pose().translation().isApprox(t.origin, kTol));
}

TEST(TrayMath, RowsAdvanceAlongXAndColumnsAlongY)
{
  const Tray t = makeTray();
  // C4 is row 2, col 3, so +2 * 0.04 in x and +3 * 0.05 in y
  const Eigen::Vector3d expected(0.45 + 0.08, -0.10 + 0.15, 0.02);
  EXPECT_TRUE(t.slotPose("C4").translation().isApprox(expected, kTol));
  EXPECT_TRUE(t.slotPose("C4").linear().isApprox(Eigen::Matrix3d::Identity(), kTol));
}

TEST(TrayMath, YawRotatesTheGridAboutA1)
{
  const Tray t = makeTray(M_PI / 2);
  // with 90 deg yaw, tray +x maps to table +y and tray +y maps to table -x
  const Eigen::Vector3d b1 = t.slotPose("B1").translation();
  const Eigen::Vector3d a2 = t.slotPose("A2").translation();
  EXPECT_NEAR(b1.x(), 0.45, kTol);
  EXPECT_NEAR(b1.y(), -0.10 + 0.04, kTol);
  EXPECT_NEAR(a2.x(), 0.45 - 0.05, kTol);
  EXPECT_NEAR(a2.y(), -0.10, kTol);
  // slot frames share the tray's orientation
  EXPECT_TRUE(t.slotPose("D6").linear().isApprox(t.pose().linear(), kTol));
}

TEST(TrayMath, RejectsSlotsOutsideTheGrid)
{
  const Tray t = makeTray();
  EXPECT_THROW(t.slotPose("E1"), std::out_of_range);
  EXPECT_THROW(t.slotPose("A7"), std::out_of_range);
  EXPECT_THROW(t.slotPose(-1, 0), std::out_of_range);
  EXPECT_NO_THROW(t.slotPose("D6"));
}

TEST(TrayMath, FootprintIncludesOuterWalls)
{
  const Tray t = makeTray();
  const auto fp = t.footprint();
  EXPECT_NEAR(fp.min.x(), 0.45 - 0.02, kTol);
  EXPECT_NEAR(fp.max.x(), 0.45 + 3.5 * 0.04, kTol);
  EXPECT_NEAR(fp.min.y(), -0.10 - 0.025, kTol);
  EXPECT_NEAR(fp.max.y(), -0.10 + 5.5 * 0.05, kTol);
}

// - resolve

TEST(Resolve, NamesMapToTheRightPoses)
{
  const Workspace ws = loadWorkspaceString(kValidYaml);
  EXPECT_TRUE(ws.resolve("tray_1").isApprox(ws.trays[0].pose(), kTol));
  EXPECT_TRUE(ws.resolve("tray_1/C4").isApprox(ws.trays[0].slotPose("C4"), kTol));
  EXPECT_TRUE(ws.resolve("bin_a").translation().isApprox(Eigen::Vector3d(0.30, 0.30, 0.02), kTol));
  const auto hole = ws.resolve("peg_fixture");
  EXPECT_TRUE(hole.translation().isApprox(Eigen::Vector3d(0.66, 0.05, 0.05), kTol));
  // yaw = pi/2, so local x points along table +y
  EXPECT_TRUE((hole.linear() * Eigen::Vector3d::UnitX()).isApprox(Eigen::Vector3d::UnitY(), 1e-9));
}

TEST(Resolve, UnknownNamesThrow)
{
  const Workspace ws = loadWorkspaceString(kValidYaml);
  EXPECT_THROW(ws.resolve("tray_2/A1"), std::invalid_argument);
  EXPECT_THROW(ws.resolve("bin_c"), std::invalid_argument);
  EXPECT_THROW(ws.resolve("bin_a/A1"), std::invalid_argument);
  EXPECT_THROW(ws.resolve("tray_1/Q9"), std::out_of_range);
  EXPECT_THROW(ws.resolve("tray_1/c4"), std::invalid_argument);
}

TEST(Resolve, BinDropSpotsSplitTheInnerFloor)
{
  const Workspace ws = loadWorkspaceString(kValidYaml);
  const Bin & b = ws.bins[0];
  EXPECT_EQ(b.spotCount(), 2);
  // the inner floor x is 0.15 - 2 * 0.005 = 0.14, so the spot centres are at -/+ 0.035 along x
  EXPECT_TRUE(ws.resolve("bin_a/1").translation().isApprox(Eigen::Vector3d(0.265, 0.30, 0.02), 1e-9));
  EXPECT_TRUE(ws.resolve("bin_a/2").translation().isApprox(Eigen::Vector3d(0.335, 0.30, 0.02), 1e-9));
  EXPECT_TRUE(ws.placePose("bin_a/2").isApprox(ws.resolve("bin_a/2"), kTol));
  EXPECT_THROW(ws.resolve("bin_a/0"), std::out_of_range);
  EXPECT_THROW(ws.resolve("bin_a/3"), std::out_of_range);
  EXPECT_THROW(ws.resolve("bin_a/"), std::invalid_argument);
  EXPECT_THROW(ws.resolve("bin_a/-1"), std::invalid_argument);

  Bin grid = b;
  grid.spot_rows = 2;
  grid.spot_cols = 3;  // row-major: spot 5 = row 1, col 1 (centre column)
  EXPECT_TRUE(grid.spotPose(5).translation().isApprox(Eigen::Vector3d(0.335, 0.30, 0.02), 1e-9));
  EXPECT_NEAR(grid.spotPose(1).translation().y(), 0.30 - 0.11 / 3, 1e-9);
  EXPECT_EQ(ws.findBin("bin_a"), &ws.bins[0]);
  EXPECT_EQ(ws.findBin("tray_1"), nullptr);
}

TEST(Resolve, PlacePoseIsWhereTheObjectBottomRests)
{
  const Workspace ws = loadWorkspaceString(kValidYaml);
  // a slot: the pocket floor, top surface (0.02) minus slot_depth (0.015)
  const auto slot = ws.placePose("tray_1/C4");
  EXPECT_TRUE(slot.translation().isApprox(Eigen::Vector3d(0.53, 0.02, 0.005), kTol));
  EXPECT_TRUE(slot.linear().isApprox(ws.trays[0].pose().linear(), kTol));
  // a bin: the floor centre
  EXPECT_TRUE(ws.placePose("bin_a").isApprox(ws.resolve("bin_a"), kTol));
  // a fixture: the bottom of the hole, along the hole axis
  EXPECT_TRUE(
    ws.placePose("peg_fixture").translation().isApprox(Eigen::Vector3d(0.66, 0.05, 0.02), 1e-9));
  EXPECT_THROW(ws.placePose("tray_1"), std::invalid_argument);
  // a one-slot tray is its own target
  Workspace one = ws;
  one.trays[0].rows = 1;
  one.trays[0].cols = 1;
  EXPECT_TRUE(one.placePose("tray_1").isApprox(one.placePose("tray_1/A1"), kTol));
  EXPECT_THROW(ws.placePose("tray_1/E1"), std::out_of_range);
  EXPECT_THROW(ws.placePose("nope"), std::invalid_argument);
}

TEST(Resolve, AllFramesCoversEverySlot)
{
  const Workspace ws = loadWorkspaceString(kValidYaml);
  const auto frames = ws.allFrames();
  EXPECT_EQ(frames.size(), 1u + 24u + 1u + 1u);
  EXPECT_EQ(Workspace::frameId("tray_1/C4"), "tray_1_C4");
  for (const auto & f : frames) {
    EXPECT_TRUE(ws.resolve(f.target).isApprox(f.pose, kTol)) << f.target;
  }
}

// - YAML loading

TEST(Yaml, LoadsAllFields)
{
  const Workspace ws = loadWorkspaceString(kValidYaml);
  EXPECT_EQ(ws.frame, "table");
  ASSERT_EQ(ws.trays.size(), 1u);
  EXPECT_EQ(ws.trays[0].name, "tray_1");
  EXPECT_EQ(ws.trays[0].rows, 4);
  EXPECT_EQ(ws.trays[0].cols, 6);
  EXPECT_DOUBLE_EQ(ws.trays[0].slot_size, 0.034);
  ASSERT_EQ(ws.bins.size(), 1u);
  EXPECT_DOUBLE_EQ(ws.bins[0].size.z(), 0.08);
  ASSERT_EQ(ws.fixtures.size(), 1u);
  EXPECT_DOUBLE_EQ(ws.fixtures[0].block_size, 0.08);  // default
  EXPECT_TRUE(ws.validate().empty());
}

TEST(Yaml, ReportsTheOffendingKey)
{
  const auto expect_error = [](const std::string & yaml, const std::string & needle) {
      try {
        loadWorkspaceString(yaml);
        ADD_FAILURE() << "expected an error mentioning '" << needle << "'";
      } catch (const std::runtime_error & e) {
        EXPECT_NE(std::string(e.what()).find(needle), std::string::npos) << e.what();
      }
    };
  expect_error("trays: {t: {rows: 4}}", "trays.t: missing required key 'origin'");
  expect_error(
    "trays: {t: {origin: [0, 0], rows: 1, cols: 1, pitch: [1, 1], slot_size: 0.5, "
    "slot_depth: 0.1}}",
    "trays.t.origin: expected a list of 3 numbers");
  expect_error("bins: {b: {center: [0, 0, x], size: [1, 1, 1]}}", "bins.b.center[2]");
  expect_error("trays: [1, 2]", "trays: expected a mapping");
  expect_error("frame: [unclosed", "YAML error");
}

// - validation

TEST(Validate, AcceptsTheProjectWorkspace)
{
  EXPECT_TRUE(loadWorkspaceString(kValidYaml).validate().empty());
}

TEST(Validate, DetectsOverlappingSlots)
{
  Workspace ws;
  ws.trays.push_back(makeTray());
  ws.trays[0].slot_size = 0.045;  // > 0.04 row pitch
  EXPECT_TRUE(contains(ws.validate(), "slots overlap"));
}

TEST(Validate, DetectsOverlappingObjects)
{
  Workspace ws;
  ws.trays.push_back(makeTray());
  Bin b;
  b.name = "bin_a";
  b.center = {0.50, 0.0, 0.02};  // on top of the tray
  b.size = {0.10, 0.10, 0.08};
  ws.bins.push_back(b);
  EXPECT_TRUE(contains(ws.validate(), "tray 'tray_1' overlaps bin 'bin_a'"));

  ws.bins[0].center = {0.20, 0.40, 0.02};  // clear of the tray
  EXPECT_TRUE(ws.validate().empty());
}

TEST(Validate, DetectsRotatedTrayOverlap)
{
  Workspace ws;
  ws.trays.push_back(makeTray(0.0));
  Fixture f;
  f.name = "fx";
  f.hole_pose.translation() = Eigen::Vector3d(0.40, 0.30, 0.05);
  f.hole_diameter = 0.02;
  f.insert_depth = 0.03;
  ws.fixtures.push_back(f);
  EXPECT_TRUE(ws.validate().empty());
  // Rotated 90 deg about A1 at (0.60, 0.20), the grid spans x 0.325..0.625 and y 0.18..0.34,
  // which covers the fixture block (x 0.36..0.44, y 0.26..0.34).
  ws.trays[0].yaw = M_PI / 2;
  ws.trays[0].origin = {0.60, 0.20, 0.02};
  EXPECT_TRUE(contains(ws.validate(), "overlaps fixture 'fx'"));
}

TEST(Validate, DetectsDuplicateAndInvalidNames)
{
  Workspace ws;
  ws.trays.push_back(makeTray());
  Bin b;
  b.name = "tray_1";
  b.center = {0.2, 0.5, 0.02};
  b.size = {0.1, 0.1, 0.08};
  ws.bins.push_back(b);
  Bin slash = b;
  slash.name = "bins/a";
  slash.center = {0.2, -0.5, 0.02};
  ws.bins.push_back(slash);
  const auto errors = ws.validate();
  EXPECT_TRUE(contains(errors, "duplicate name 'tray_1'"));
  EXPECT_TRUE(contains(errors, "must not contain '/'"));
}

TEST(Validate, DetectsBadDimensions)
{
  Workspace ws;
  Tray t = makeTray();
  t.rows = 27;
  t.pitch = {0.0, 0.04};
  t.slot_depth = 0.03;  // deeper than the 0.02 tray
  ws.trays.push_back(t);
  Fixture f;
  f.name = "fx";
  f.hole_pose.translation() = Eigen::Vector3d(0.0, 0.5, 0.02);
  f.hole_diameter = 0.1;  // wider than the block
  f.insert_depth = 0.03;  // deeper than the block is tall
  ws.fixtures.push_back(f);
  const auto errors = ws.validate();
  EXPECT_TRUE(contains(errors, "rows must be 1..26"));
  EXPECT_TRUE(contains(errors, "pitch must be positive"));
  EXPECT_TRUE(contains(errors, "slot_depth must be less than the tray height"));
  EXPECT_TRUE(contains(errors, "block_size must exceed hole_diameter"));
  EXPECT_TRUE(contains(errors, "insert_depth must be less than the hole height"));
}

TEST(Validate, ReportsEveryProblemAtOnce)
{
  Workspace ws;
  ws.frame = "";
  Tray t = makeTray();
  t.slot_size = -1;
  ws.trays.push_back(t);
  EXPECT_GE(ws.validate().size(), 2u);
}
