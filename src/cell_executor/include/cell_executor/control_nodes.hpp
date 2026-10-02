// These are the nodes that give a generated tree its task structure: Step, which reports progress
// and applies the failure policy; ForEachObject, which loops over detected objects; WatchObjects,
// which keeps looking and handles each object as it comes to meet a condition; RepeatUntilObject,
// which repeats its child until an object meets a condition; and the ObjectIs and IsHolding
// conditions. They only touch the RunContext and never call a skill themselves (WatchObjects has
// its detection as a child), so test_tree runs them with stub skills and no robot.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <deque>
#include <set>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>

#include "cell_executor/run_context.hpp"

namespace cell_executor
{

// One top-level task step. It reports progress, and when its child fails it applies the policy:
// skip_step returns SUCCESS so the task carries on, safe_stop and abort return FAILURE.
class StepNode : public BT::DecoratorNode
{
public:
  StepNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  std::shared_ptr<RunContext> ctx_;
  double started_ = 0.0;
};

// Runs its child once per detected object that matches `filter` (at most `max`), with the
// blackboard entries "item" (the object id) and "index" (1-based) set. The objects are chosen
// when the loop starts, and when that list runs out the world is asked again, so an object that
// a look inside the loop found for the first time is handled too (a one-frame look at the start
// of a readout run missed a tube that the six-frame look inside the loop saw). When the child
// fails for an object, skip_step moves on to the next object and any other policy fails the loop.
class ForEachObjectNode : public BT::DecoratorNode
{
public:
  ForEachObjectNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  void halt() override;
  // appends matching objects the loop has not visited yet; false when there are none
  bool addNewItems();
  std::shared_ptr<RunContext> ctx_;
  ObjectFilter filter_;
  int max_ = 0;
  std::vector<std::string> items_;
  std::size_t current_ = 0;
  bool item_started_ = false;
};

// SUCCESS while the executor believes the gripper holds an object
class IsHoldingNode : public BT::ConditionNode
{
public:
  IsHoldingNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx);
  static BT::PortsList providedPorts() {return {};}

private:
  BT::NodeStatus tick() override;
  std::shared_ptr<RunContext> ctx_;
};

// The lab's watch loop. Its first child looks (a DetectObjects node) and its second handles one
// object, with "item" and "index" set as in ForEachObject. On the first look it takes every
// detected object that matches `filter` as watched; after every look it handles each watched
// object that meets `when` (e.g. color=brown,green), one at a time and looking again after each,
// because more may have changed meanwhile; when none is ready it waits `period` seconds and looks
// again. It succeeds once every watched object has been handled (or put away) and fails when
// `timeout` seconds pass first. A failed look is tried again after `period`, and only
// `max_failed_looks` in a row fail the watch, because a watch makes dozens of looks and one lost
// camera frame used to end it with every vial still unsorted. A failed object is skipped under
// skip_step, as in ForEachObject.
class WatchObjectsNode : public BT::ControlNode
{
public:
  WatchObjectsNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx);
  static BT::PortsList providedPorts();

private:
  enum class Phase { Look, Wait, Handle };
  BT::NodeStatus tick() override;
  void halt() override;
  std::shared_ptr<RunContext> ctx_;
  Phase phase_ = Phase::Look;
  std::vector<std::string> watched_;
  std::set<std::string> done_;
  std::deque<std::string> queue_;
  double started_ = 0.0;
  double next_look_ = 0.0;
  int handled_ = 0;
  int failed_looks_ = 0;
  bool item_started_ = false;
};

// Repeats its child until the object `item` meets `condition`, checking before every round. After
// `max` rounds without it, it fails, or with on_max="continue" logs it and succeeds, so the task
// can deal with the object itself (the titration sends a vial that never turned to waste rather
// than leaving it in the dispenser, where it would block the next one). The child is expected to
// change the object and look at it again (in the lab: dispense, wait, detect), since the
// condition is only ever checked against what was last detected.
class RepeatUntilObjectNode : public BT::DecoratorNode
{
public:
  RepeatUntilObjectNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  void halt() override;
  std::shared_ptr<RunContext> ctx_;
  int rounds_ = 0;
  bool child_running_ = false;
};

// SUCCESS when the object `item` meets `condition` as last detected; the test of an if step
class ObjectIsNode : public BT::ConditionNode
{
public:
  ObjectIsNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  std::shared_ptr<RunContext> ctx_;
};

// registers every node in this file with the factory, all sharing `ctx`
void registerControlNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<RunContext> ctx);

}  // namespace cell_executor
