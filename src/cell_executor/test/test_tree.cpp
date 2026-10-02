// These are the tests for the task logic. They run generated trees with the real control nodes
// (Step, ForEachObject, IsHolding and the SafeStop subtree from trees/) and stub skills that
// record their calls, so the order of calls, the bin drop spots and each failure policy can be
// checked without ROS or a robot. The workspace below has the same trays, bins and fixture as
// cell_description/config/workspace.yaml, including the two drop spots per bin that exist
// because two cubes placed at a bin centre landed on each other. Run them with
// colcon test --packages-select cell_executor.
#include <algorithm>
#include <set>
#include <utility>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cell_executor/control_nodes.hpp"
#include "cell_executor/task.hpp"

using namespace cell_executor;

namespace
{

const char * kWorkspace = R"(
frame: table
trays:
  tray_1: {origin: [0.45, -0.10, 0.02], rows: 4, cols: 6, pitch: [0.04, 0.04],
           slot_size: 0.034, slot_depth: 0.015}
  dispenser: {origin: [0.62, 0.0, 0.02], rows: 1, cols: 1, pitch: [0.05, 0.05],
              slot_size: 0.034, slot_depth: 0.015}
bins:
  bin_a: {center: [0.30, 0.30, 0.02], size: [0.15, 0.12, 0.08], spots: [2, 1]}
  bin_b: {center: [0.50, 0.30, 0.02], size: [0.15, 0.12, 0.08], spots: [2, 1]}
fixtures:
  peg_fixture: {hole_pose: [0.66, 0.05, 0.05, 0, 0, 0], hole_diameter: 0.022,
                insert_depth: 0.030}
)";

DetectedObject object(const std::string & id, const std::string & type, const std::string & cls,
  const std::string & color)
{
  DetectedObject o;
  o.id = id;
  o.type = type;
  o.object_class = cls;
  o.color = color;
  return o;
}

// a 50 mm vial standing at (x, y), named as the detector would name it in that colour
DetectedObject vial(const std::string & color, int n, double x, double y)
{
  DetectedObject o = object(
    "vial_" + color + "_" + std::to_string(n), "vial_" + color, "vial", color);
  o.pose.pose.position.x = x;
  o.pose.pose.position.y = y;
  o.pose.pose.position.z = 0.03;
  o.size.x = o.size.y = 0.03;
  o.size.z = 0.05;
  return o;
}

// The default scene's objects as the detector names them. red_cube_1 comes before red_cube_0
// here, and the executor still handles them in id order.
std::vector<DetectedObject> scene()
{
  std::vector<DetectedObject> objects = {
    object("blue_cylinder_0", "blue_cylinder", "cylinder", "blue"),
    object("blue_cylinder_1", "blue_cylinder", "cylinder", "blue"),
    object("blue_cylinder_2", "blue_cylinder", "cylinder", "blue"),
    object("blue_cylinder_3", "blue_cylinder", "cylinder", "blue"),
    object("green_cube_0", "green_cube", "cube", "green"),
    object("red_cube_1", "red_cube", "cube", "red"),
    object("red_cube_0", "red_cube", "cube", "red"),
    object("yellow_peg_0", "yellow_peg", "peg", "yellow"),
  };
  // 10 cm apart along x, so the world's position matching tells them apart
  for (std::size_t i = 0; i < objects.size(); ++i) {
    objects[i].pose.pose.position.x = 0.1 * static_cast<double>(i);
  }
  return objects;
}

class TreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    workspace_ = cell_workspace::loadWorkspaceString(kWorkspace);
    ctx_ = std::make_shared<RunContext>();
    ctx_->workspace = &workspace_;
  }

  // Registers the control nodes and stub skills, builds the tree for `yaml` and ticks it to the
  // end. Each call is recorded as "Skill arg".
  BT::NodeStatus run(const std::string & yaml, FailurePolicy policy)
  {
    Task task = parseTask(yaml);
    task.on_failure = policy;
    BT::BehaviorTreeFactory factory;
    registerControlNodes(factory, ctx_);

    const auto expand = [](BT::TreeNode & node, const std::string & port) {
        const std::string text = node.getInput<std::string>(port).value();
        return expandVariables(text, [&](const std::string & name) -> std::optional<std::string> {
                 std::string value;
                 if (std::as_const(node).config().blackboard->get(name, value)) {
                   return value;
                 }
                 return std::nullopt;
               });
      };
    const auto outcome = [this](const std::string & call) {
        calls_.push_back(call);
        if (fail_once_.erase(call)) {
          return BT::NodeStatus::FAILURE;
        }
        return fail_.count(call) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
      };

    factory.registerSimpleAction("DetectObjects", [this, outcome](BT::TreeNode &) {
        const auto status = outcome("DetectObjects");
        if (status == BT::NodeStatus::SUCCESS) {
          // a scripted camera: the next frame each time, then the last one again
          if (frames_.empty()) {
            ctx_->world.updateDetections(scene(), {});
          } else {
            ctx_->world.updateDetections(frames_[std::min(looks_++, frames_.size() - 1)], {});
          }
        }
        return status;
      }, {BT::InputPort<std::string>("class"), BT::InputPort<std::string>("color"),
        BT::InputPort<int>("frames")});
    factory.registerSimpleAction("Dispense", [outcome](BT::TreeNode &) {
        return outcome("Dispense");
      });
    factory.registerSimpleAction("Pick", [this, expand, outcome](BT::TreeNode & node) {
        const auto found = ctx_->world.find(expand(node, "object"));
        if (!found) {
          calls_.push_back("Pick ?" + expand(node, "object"));
          return BT::NodeStatus::FAILURE;
        }
        const auto status = outcome("Pick " + found->id);
        if (status == BT::NodeStatus::SUCCESS) {
          ctx_->world.held = found->id;
          ctx_->world.unmarkHandled(found->id);
        }
        return status;
      }, {BT::InputPort<std::string>("object"), BT::InputPort<std::string>("grasp")});
    factory.registerSimpleAction("Place", [this, expand, outcome](BT::TreeNode & node) {
        const std::string target = ctx_->allocateTarget(expand(node, "target"));
        const auto status = outcome("Place " + target);
        if (status == BT::NodeStatus::SUCCESS) {
          ctx_->world.moveTo(*ctx_->world.held, ctx_->workspace->placePose(target).translation() +
            Eigen::Vector3d(0, 0, 0.025));
          ctx_->world.markHandled(*ctx_->world.held);
          ctx_->world.held.reset();
        }
        return status;
      }, {BT::InputPort<std::string>("target"), BT::InputPort<std::string>("compliance")});
    factory.registerSimpleAction("Insert", [this, expand, outcome](BT::TreeNode & node) {
        const auto status = outcome("Insert " + expand(node, "target"));
        if (status == BT::NodeStatus::SUCCESS) {
          ctx_->world.held.reset();
        }
        return status;
      }, {BT::InputPort<std::string>("target"), BT::InputPort<double>("max_force"),
        BT::InputPort<int>("max_attempts")});
    factory.registerSimpleAction("MoveToNamedPose", [expand, outcome](BT::TreeNode & node) {
        return outcome("MoveToNamedPose " + expand(node, "pose"));
      }, {BT::InputPort<std::string>("pose")});

    factory.registerBehaviorTreeFromFile(std::string(TREES_DIR) + "/safe_stop.xml");
    factory.registerBehaviorTreeFromText(toBehaviorTreeXml(task));
    BT::Tree tree = factory.createTree("Task");
    return tree.tickWhileRunning(std::chrono::milliseconds(0));
  }

  std::string referenceTask() const
  {
    return R"(
name: sort_and_insert
steps:
  - detect: {}
  - for_each:
      filter: {color: red}
      do:
        - pick: {object: $item}
        - place: {target: bin_a}
  - for_each:
      filter: {class: cylinder}
      max: 4
      do:
        - pick: {object: $item}
        - place: {target: tray_1/C$index}
  - pick: {object: peg_0}
  - insert: {target: peg_fixture, max_attempts: 3}
  - move_to: {pose: home}
)";
  }

  cell_workspace::Workspace workspace_;
  std::shared_ptr<RunContext> ctx_;
  std::vector<std::string> calls_;
  std::set<std::string> fail_;  // calls that fail
  std::multiset<std::string> fail_once_;  // calls that fail once each time they are listed
  std::vector<std::vector<DetectedObject>> frames_;  // the scripted camera, when set
  std::size_t looks_ = 0;
};

}  // namespace

TEST_F(TreeTest, ReferenceTaskRunsEveryStepInOrder)
{
  EXPECT_EQ(run(referenceTask(), FailurePolicy::SafeStop), BT::NodeStatus::SUCCESS);
  const std::vector<std::string> expected = {
    "DetectObjects",
    "Pick red_cube_0", "Place bin_a/1",
    "Pick red_cube_1", "Place bin_a/2",
    "Pick blue_cylinder_0", "Place tray_1/C1",
    "Pick blue_cylinder_1", "Place tray_1/C2",
    "Pick blue_cylinder_2", "Place tray_1/C3",
    "Pick blue_cylinder_3", "Place tray_1/C4",
    "Pick yellow_peg_0", "Insert peg_fixture",
    "MoveToNamedPose home",
  };
  EXPECT_EQ(calls_, expected);
  EXPECT_EQ(ctx_->steps_succeeded, 6);
  EXPECT_EQ(ctx_->failed_step, 0);
  EXPECT_FALSE(ctx_->safe_stopped);
}

TEST_F(TreeTest, SkipStepSkipsOnlyTheFailedObject)
{
  fail_ = {"Pick blue_cylinder_1"};
  EXPECT_EQ(run(referenceTask(), FailurePolicy::SkipStep), BT::NodeStatus::SUCCESS);
  // C2 stays empty because the index follows the object; the other cylinders and the peg still go
  const std::vector<std::string> cylinders(calls_.begin() + 5, calls_.begin() + 12);
  EXPECT_EQ(cylinders, (std::vector<std::string>{
    "Pick blue_cylinder_0", "Place tray_1/C1", "Pick blue_cylinder_1",
    "Pick blue_cylinder_2", "Place tray_1/C3", "Pick blue_cylinder_3", "Place tray_1/C4"}));
  EXPECT_EQ(calls_.back(), "MoveToNamedPose home");
  EXPECT_EQ(ctx_->items_skipped, 1);
  EXPECT_EQ(ctx_->steps_succeeded, 6);
}

TEST_F(TreeTest, SkipStepSkipsAFailedTopLevelStep)
{
  fail_ = {"Insert peg_fixture"};
  EXPECT_EQ(run(referenceTask(), FailurePolicy::SkipStep), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls_.back(), "MoveToNamedPose home");
  EXPECT_EQ(ctx_->steps_skipped, 1);
  EXPECT_EQ(ctx_->steps_succeeded, 5);
}

TEST_F(TreeTest, SafeStopPutsTheHeldObjectDownAndGoesHome)
{
  fail_ = {"Place bin_a/1"};  // still holding red_cube_0
  EXPECT_EQ(run(referenceTask(), FailurePolicy::SafeStop), BT::NodeStatus::FAILURE);
  EXPECT_EQ(calls_, (std::vector<std::string>{
    "DetectObjects", "Pick red_cube_0", "Place bin_a/1",
    "Place bin_b/1", "MoveToNamedPose home"}));
  EXPECT_TRUE(ctx_->safe_stopped);
  EXPECT_EQ(ctx_->failed_step, 2);
  EXPECT_FALSE(ctx_->world.held);
}

TEST_F(TreeTest, SafeStopWithEmptyGripperJustGoesHome)
{
  fail_ = {"Pick yellow_peg_0"};
  EXPECT_EQ(run(referenceTask(), FailurePolicy::SafeStop), BT::NodeStatus::FAILURE);
  const std::vector<std::string> tail(calls_.end() - 2, calls_.end());
  EXPECT_EQ(tail, (std::vector<std::string>{"Pick yellow_peg_0", "MoveToNamedPose home"}));
  EXPECT_EQ(ctx_->failed_step, 4);
}

TEST_F(TreeTest, AbortStopsImmediately)
{
  fail_ = {"Pick red_cube_1"};
  EXPECT_EQ(run(referenceTask(), FailurePolicy::Abort), BT::NodeStatus::FAILURE);
  EXPECT_EQ(calls_.back(), "Pick red_cube_1");
  EXPECT_EQ(ctx_->failed_step, 2);
  EXPECT_FALSE(ctx_->safe_stopped);
}

TEST_F(TreeTest, ForEachWithNoMatchesSucceeds)
{
  const std::string yaml =
    "name: t\nsteps:\n  - detect\n  - for_each:\n      filter: {color: purple}\n"
    "      do: [{pick: $item}]\n  - move_to: home\n";
  EXPECT_EQ(run(yaml, FailurePolicy::Abort), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls_, (std::vector<std::string>{"DetectObjects", "MoveToNamedPose home"}));
}

TEST_F(TreeTest, BinSpotsWrapWhenFull)
{
  EXPECT_EQ(ctx_->allocateTarget("bin_a"), "bin_a/1");
  EXPECT_EQ(ctx_->allocateTarget("bin_a"), "bin_a/2");
  EXPECT_EQ(ctx_->allocateTarget("bin_a"), "bin_a/1");
  EXPECT_EQ(ctx_->allocateTarget("bin_b"), "bin_b/1");
  EXPECT_EQ(ctx_->allocateTarget("bin_a/2"), "bin_a/2");
  EXPECT_EQ(ctx_->allocateTarget("tray_1/C1"), "tray_1/C1");
}

TEST(World, FindsByIdOrAliasAndSelectsUnhandledObjects)
{
  World world;
  world.updateDetections(scene(), {});
  EXPECT_EQ(world.find("red_cube_1")->id, "red_cube_1");
  EXPECT_EQ(world.find("peg_0")->id, "yellow_peg_0");      // class alias
  EXPECT_EQ(world.find("cube_1")->id, "red_cube_0");       // cubes by id: green_cube_0, red_cube_0
  EXPECT_EQ(world.find("cylinder_3")->id, "blue_cylinder_3");
  EXPECT_FALSE(world.find("peg_1"));
  EXPECT_FALSE(world.find("nothing"));

  EXPECT_EQ(world.select({{"color", "red"}}),
    (std::vector<std::string>{"red_cube_0", "red_cube_1"}));
  EXPECT_EQ(world.select({{"class", "cylinder"}}, 2).size(), 2u);
  world.markHandled("red_cube_0");
  world.held = "red_cube_1";
  EXPECT_TRUE(world.select({{"color", "red"}}).empty());
  EXPECT_EQ(world.select({}).size(), 6u);

  // A filtered detection drops only the matching objects it did not see again, and never the one
  // in the gripper, which the camera cannot see: red_cube_0 is seen where it was, red_cube_1 is
  // held, so all eight remain.
  world.updateDetections({scene()[6]}, {{"color", "red"}});
  EXPECT_EQ(world.detections().size(), 8u);
  world.held.reset();
  world.updateDetections({scene()[6]}, {{"color", "red"}});
  EXPECT_EQ(world.detections().size(), 7u);  // red_cube_1 is gone now that it is not held
}

// - the lab's watch, if and repeat_until

TEST_F(TreeTest, WatchHandlesEachObjectWhenItChangesAndKeepsItsId)
{
  // vial 1 turns green first; the detector then calls the remaining blue vial vial_blue_0,
  // which the world must still know as vial_blue_1
  frames_ = {
    {vial("blue", 0, 0.40, -0.36), vial("blue", 1, 0.40, -0.30)},
    {vial("blue", 0, 0.40, -0.36), vial("blue", 1, 0.40, -0.30)},
    {vial("green", 0, 0.40, -0.36), vial("blue", 0, 0.40, -0.30)},
    {vial("brown", 0, 0.40, -0.30)},
  };
  const std::string yaml = R"(
name: assay
steps:
  - detect
  - watch:
      filter: {class: vial}
      when: {color: [green, brown]}
      period: 0.01
      timeout: 5
      do:
        - pick: $item
        - if:
            color: green
            then: [{place: tray_1}]
            else: [{place: bin_b}]
)";
  EXPECT_EQ(run(yaml, FailurePolicy::Abort), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls_, (std::vector<std::string>{
    "DetectObjects", "DetectObjects", "DetectObjects",
    "Pick vial_blue_0", "Place tray_1/A1",
    "DetectObjects", "Pick vial_blue_1", "Place bin_b/1",
    "DetectObjects"}));
}

TEST_F(TreeTest, WatchLooksAgainAfterAFailedLook)
{
  frames_ = {{vial("green", 0, 0.40, -0.36)}};
  fail_once_ = {"DetectObjects"};  // the watch's first look loses its camera frame
  const std::string yaml =
    "name: t\nsteps:\n  - watch:\n      filter: {class: vial}\n"
    "      when: {color: green}\n      period: 0.01\n      timeout: 5\n"
    "      do: [{pick: $item}, {place: tray_1}]\n";
  EXPECT_EQ(run(yaml, FailurePolicy::Abort), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls_, (std::vector<std::string>{
    "DetectObjects", "DetectObjects", "Pick vial_green_0", "Place tray_1/A1", "DetectObjects"}));
}

TEST_F(TreeTest, WatchTimesOutWhenNothingChanges)
{
  frames_ = {{vial("blue", 0, 0.40, -0.36)}};
  const std::string yaml =
    "name: t\nsteps:\n  - watch:\n      filter: {class: vial}\n"
    "      when: {color: green}\n      period: 0.01\n      timeout: 0.05\n"
    "      do: [{pick: $item}]\n";
  EXPECT_EQ(run(yaml, FailurePolicy::Abort), BT::NodeStatus::FAILURE);
  EXPECT_NE(ctx_->last_error.find("timed out"), std::string::npos) << ctx_->last_error;
  EXPECT_EQ(std::count(calls_.begin(), calls_.end(), "Pick vial_blue_0"), 0);
}

TEST_F(TreeTest, RepeatUntilDosesUntilTheColourIsReached)
{
  // the vial is set in the dispenser, then every look after a dose shows the next frame
  const double dx = 0.62, dy = 0.0;
  frames_ = {
    {vial("yellow", 0, 0.40, -0.36)},
    {vial("orange", 0, dx, dy)},
    {vial("orange", 0, dx, dy)},
    {vial("red", 0, dx, dy)},
  };
  const std::string yaml = R"(
name: titration
steps:
  - detect
  - for_each:
      filter: {class: vial}
      do:
        - pick: $item
        - place: dispenser
        - repeat_until:
            color: red
            max: 6
            do: [dispense, {wait: 0.001}, detect]
        - pick: $item
        - place: tray_1
)";
  EXPECT_EQ(run(yaml, FailurePolicy::Abort), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(std::count(calls_.begin(), calls_.end(), "Dispense"), 3);
  EXPECT_EQ(calls_.back(), "Place tray_1/A1");
  EXPECT_EQ(ctx_->world.find("vial_yellow_0")->color, "red");
}

TEST_F(TreeTest, RepeatUntilGivesUpAfterMaxRounds)
{
  frames_ = {{vial("yellow", 0, 0.62, 0.0)}};
  const std::string yaml = R"(
name: t
steps:
  - detect
  - repeat_until:
      item: vial_yellow_0
      color: red
      max: 2
      do: [dispense, detect]
)";
  EXPECT_EQ(run(yaml, FailurePolicy::Abort), BT::NodeStatus::FAILURE);
  EXPECT_EQ(std::count(calls_.begin(), calls_.end(), "Dispense"), 2);
  EXPECT_NE(ctx_->last_error.find("did not meet color=red after 2 rounds"), std::string::npos)
    << ctx_->last_error;
}

TEST_F(TreeTest, RepeatUntilCanCarryOnAfterMaxRounds)
{
  frames_ = {{vial("yellow", 0, 0.62, 0.0)}};
  const std::string yaml = R"(
name: t
steps:
  - detect
  - repeat_until:
      item: vial_yellow_0
      color: red
      max: 2
      on_max: continue
      do: [dispense, detect]
  - move_to: home
)";
  EXPECT_EQ(run(yaml, FailurePolicy::Abort), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls_.back(), "MoveToNamedPose home");
}

TEST_F(TreeTest, ReadoutDecidesByHueAndLooksTwiceAtABorderlineTube)
{
  // Tube 0 is clearly negative and tube 1 borderline on both looks. Tube 2 is first seen by the
  // second look, which the loop must still handle (a positive, so it goes to bin_a). Each frame
  // shows what the camera would: a tube that was put away is no longer where it stood.
  auto tube = [](int n, double y, double hue) {
      DetectedObject o = vial("readout", n, 0.40, y);
      o.object_class = "tube";
      o.hue = static_cast<float>(hue);
      return o;
    };
  frames_ = {
    {tube(0, -0.36, 60.0), tube(1, -0.30, 110.0)},
    {tube(1, -0.30, 110.0), tube(2, -0.24, 170.0)},
    {tube(2, -0.24, 170.0)}};
  const std::string yaml = R"(
name: readout
steps:
  - detect
  - for_each:
      filter: {class: tube}
      do:
        - if:
            hue: "<75"
            then: [{pick: $item}, {place: tray_1}]
            else:
              - detect: {frames: 6}
              - if:
                  hue: ["<=125", ">=95"]
                  then: [{pick: $item}, {place: bin_b}]
                  else: [{pick: $item}, {place: bin_a}]
)";
  EXPECT_EQ(run(yaml, FailurePolicy::Abort), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(calls_, (std::vector<std::string>{
    "DetectObjects", "Pick vial_readout_0", "Place tray_1/A1",
    "DetectObjects", "Pick vial_readout_1", "Place bin_b/1",
    "DetectObjects", "Pick vial_readout_2", "Place bin_a/1"}));
}

TEST_F(TreeTest, WatchHarvestsADishOnceItsColonyIsLargeEnough)
{
  auto dish = [](double area_mm2) {
      DetectedObject o = vial("magenta", 0, 0.58, 0.12);
      o.object_class = "dish";
      o.inner_area = static_cast<float>(area_mm2 * 1e-6);
      return o;
    };
  frames_ = {{dish(50)}, {dish(201)}, {dish(452)}, {dish(707)}};
  const std::string yaml =
    "name: g\nsteps:\n  - watch:\n      filter: {class: dish}\n"
    "      when: {area: \">=600\"}\n      period: 0.01\n      timeout: 5\n"
    "      do: [{pick: $item}, {place: tray_1}]\n";
  EXPECT_EQ(run(yaml, FailurePolicy::Abort), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(std::count(calls_.begin(), calls_.end(), "DetectObjects"), 5);
  EXPECT_EQ(std::count(calls_.begin(), calls_.end(), "Pick vial_magenta_0"), 1);
}

TEST(World, TracksObjectsThroughColourChanges)
{
  World world;
  world.updateDetections({vial("blue", 0, 0.40, -0.36), vial("blue", 1, 0.40, -0.30)}, {});
  const auto changes =
    world.updateDetections({vial("blue", 0, 0.40, -0.30), vial("green", 0, 0.401, -0.36)}, {});
  ASSERT_EQ(changes.size(), 1u);
  EXPECT_EQ(changes[0].id, "vial_blue_0");
  EXPECT_EQ(changes[0].to, "green");
  EXPECT_EQ(world.find("vial_blue_0")->color, "green");
  EXPECT_EQ(world.find("vial_blue_1")->color, "blue");
  EXPECT_TRUE(world.meets("vial_blue_0", {{"color", {"green", "brown"}}}));
  EXPECT_FALSE(world.meets("vial_blue_1", {{"color", {"green", "brown"}}}));
  // something new where nothing was gets its own id, never one already taken
  world.updateDetections({vial("blue", 0, 0.60, 0.20)}, {{"color", "red"}});
  EXPECT_EQ(world.detections().size(), 3u);
  EXPECT_EQ(world.detections().back().id, "vial_blue_2");
}

TEST_F(TreeTest, TraySlotsAreAllocatedInRowOrder)
{
  EXPECT_EQ(ctx_->allocateTarget("tray_1"), "tray_1/A1");
  EXPECT_EQ(ctx_->allocateTarget("tray_1"), "tray_1/A2");
  for (int i = 0; i < 4; ++i) {
    ctx_->allocateTarget("tray_1");
  }
  EXPECT_EQ(ctx_->allocateTarget("tray_1"), "tray_1/B1");
  EXPECT_EQ(ctx_->allocateTarget("dispenser"), "dispenser");
  EXPECT_EQ(ctx_->allocateTarget("tray_1/C"), "tray_1/C1");
  EXPECT_EQ(ctx_->allocateTarget("tray_1/C"), "tray_1/C2");
  EXPECT_EQ(ctx_->allocateTarget("tray_1/C4"), "tray_1/C4");
}
