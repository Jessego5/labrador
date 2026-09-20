// These are the nodes that give a generated tree its task structure: Step, which reports progress
// and applies the failure policy, ForEachObject, which loops over detected objects, and the
// IsHolding condition used by the SafeStop subtree. They only touch the RunContext and never call
// a skill themselves, so test_tree runs them with stub skills and no robot.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
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
// when the loop starts. When the child fails for an object, skip_step moves on to the next
// object and any other policy fails the loop.
class ForEachObjectNode : public BT::DecoratorNode
{
public:
  ForEachObjectNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx);
  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  void halt() override;
  std::shared_ptr<RunContext> ctx_;
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

// registers Step, ForEachObject and IsHolding with the factory, all sharing `ctx`
void registerControlNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<RunContext> ctx);

}  // namespace cell_executor
