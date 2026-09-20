// These are the tests for the task logic. They run generated trees with the real control nodes
// (Step, ForEachObject, IsHolding and the SafeStop subtree from trees/) and stub skills that
// record their calls, so the order of calls, the bin drop spots and each failure policy can be
// checked without ROS or a robot. The workspace below has the same trays, bins and fixture as
// cell_description/config/workspace.yaml, including the two drop spots per bin that exist
// because two cubes placed at a bin centre landed on each other. Run them with
// colcon test --packages-select cell_executor.
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

// The default scene's objects as the detector names them. red_cube_1 comes before red_cube_0
// here, and the executor still handles them in id order.
std::vector<DetectedObject> scene()
{
  return {
    object("blue_cylinder_0", "blue_cylinder", "cylinder", "blue"),
    object("blue_cylinder_1", "blue_cylinder", "cylinder", "blue"),
    object("blue_cylinder_2", "blue_cylinder", "cylinder", "blue"),
    object("blue_cylinder_3", "blue_cylinder", "cylinder", "blue"),
    object("green_cube_0", "green_cube", "cube", "green"),
    object("red_cube_1", "red_cube", "cube", "red"),
    object("red_cube_0", "red_cube", "cube", "red"),
    object("yellow_peg_0", "yellow_peg", "peg", "yellow"),
  };
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
        return fail_.count(call) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
      };

    factory.registerSimpleAction("DetectObjects", [this, outcome](BT::TreeNode &) {
        const auto status = outcome("DetectObjects");
        if (status == BT::NodeStatus::SUCCESS) {
          ctx_->world.updateDetections(scene(), {});
        }
        return status;
      }, {BT::InputPort<std::string>("class"), BT::InputPort<std::string>("color")});
    factory.registerSimpleAction("Pick", [this, expand, outcome](BT::TreeNode & node) {
        const auto found = ctx_->world.find(expand(node, "object"));
        if (!found) {
          calls_.push_back("Pick ?" + expand(node, "object"));
          return BT::NodeStatus::FAILURE;
        }
        const auto status = outcome("Pick " + found->id);
        if (status == BT::NodeStatus::SUCCESS) {
          ctx_->world.held = found->id;
        }
        return status;
      }, {BT::InputPort<std::string>("object"), BT::InputPort<std::string>("grasp")});
    factory.registerSimpleAction("Place", [this, expand, outcome](BT::TreeNode & node) {
        const auto status = outcome("Place " + ctx_->allocateTarget(expand(node, "target")));
        if (status == BT::NodeStatus::SUCCESS) {
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

  // a filtered detection replaces only the matching objects
  world.updateDetections({object("red_cube_0", "red_cube", "cube", "red")}, {{"color", "red"}});
  EXPECT_EQ(world.detections().size(), 7u);
}
