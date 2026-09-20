// These are the behavior-tree action nodes that call the cell_skills action servers,
// DetectObjects, Pick, Place, Insert and MoveToNamedPose, one node per skill. The executor node
// registers them with the factory, and every leaf of a generated tree is one of them.
//
// They share SkillNode<ActionT>, a small asynchronous wrapper around an rclcpp_action client. It
// is the same idea as BehaviorTree.ROS2's RosActionNode, written here because BehaviorTree.ROS2
// is not packaged for Jazzy. It waits for the server, sends the goal (resending briefly while the
// skill server is still busy), polls for the result on every tick, enforces a timeout and cancels
// the goal when halted. The tick thread never blocks; the ROS executor delivers the responses on
// its own threads.
#pragma once

#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <cell_interfaces/action/detect_objects.hpp>
#include <cell_interfaces/action/insert.hpp>
#include <cell_interfaces/action/move_to_named_pose.hpp>
#include <cell_interfaces/action/pick.hpp>
#include <cell_interfaces/action/place.hpp>
#include <cell_interfaces/msg/skill_error.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "cell_executor/run_context.hpp"

namespace cell_executor
{

// "PLAN_FAILED" and so on for a cell_interfaces/SkillError code
std::string errorName(uint8_t code);

// the action clients for the skills, created once by the executor node
struct SkillClients
{
  rclcpp_action::Client<cell_interfaces::action::DetectObjects>::SharedPtr detect;
  rclcpp_action::Client<cell_interfaces::action::Pick>::SharedPtr pick;
  rclcpp_action::Client<cell_interfaces::action::Place>::SharedPtr place;
  rclcpp_action::Client<cell_interfaces::action::Insert>::SharedPtr insert;
  rclcpp_action::Client<cell_interfaces::action::MoveToNamedPose>::SharedPtr move;

  std::map<std::string, double> timeouts;  // skill name ("Pick") -> s
  double server_wait = 10.0;               // s to wait for an action server to appear
  double busy_retry = 5.0;                 // s to keep resending a rejected goal
  double ready_wait = 180.0;               // s to keep resending while a skill is NOT_READY

  double timeout(const std::string & skill) const
  {
    const auto it = timeouts.find(skill);
    return it == timeouts.end() ? 120.0 : it->second;
  }
};

namespace detail
{
template<typename T, typename = void>
struct HasPhase : std::false_type {};
template<typename T>
struct HasPhase<T, std::void_t<decltype(std::declval<T>().phase)>>: std::true_type {};
}  // namespace detail

template<class ActionT>
class SkillNode : public BT::StatefulActionNode
{
public:
  using Goal = typename ActionT::Goal;
  using Result = typename ActionT::Result;
  using Feedback = typename ActionT::Feedback;
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using SkillError = cell_interfaces::msg::SkillError;

  SkillNode(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx,
    std::shared_ptr<SkillClients> skills, typename Client::SharedPtr client, std::string skill)
  : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)), skills_(std::move(skills)),
    client_(std::move(client)), skill_(std::move(skill))
  {
  }

protected:
  // Fills in the goal. Returning an error message fails the node without sending anything, and
  // a throw does the same with the exception's message.
  virtual std::optional<std::string> makeGoal(Goal & goal) = 0;

  // Called once the skill has finished, with a null result when none came back. This is where a
  // node updates the world model and adds its own fields to the log record.
  virtual void onResult(bool success, const Result * result, Json & record)
  {
    (void)success;
    (void)result;
    (void)record;
  }

  // `text` with $item, $index and any other variable filled in from the blackboard
  std::string expand(const std::string & text) const
  {
    return expandVariables(text, [this](const std::string & name) -> std::optional<std::string> {
               std::string value;
               if (config().blackboard->get(name, value)) {
                 return value;
               }
               return std::nullopt;
             });
  }

  std::shared_ptr<RunContext> ctx_;
  std::shared_ptr<SkillClients> skills_;
  std::string target_;  // what the call acts on, for the log

private:
  enum class Stage { WaitServer, Sending, Retry, Running };

  BT::NodeStatus onStart() override
  {
    start_ = std::chrono::steady_clock::now();
    waited_ = 0.0;
    goal_ = Goal();
    target_.clear();
    handle_.reset();
    ++ctx_->skill_calls;
    ctx_->updateProgress([&](Progress & p) {
        p.skill = skill_;
        p.phase.clear();
      });
    std::optional<std::string> error;
    try {
      error = makeGoal(goal_);
    } catch (const std::exception & e) {
      error = e.what();
    }
    if (error) {
      return finish(false, SkillError::INVALID_GOAL, *error, nullptr);
    }
    ctx_->info(skill_ + (target_.empty() ? "" : " " + target_));
    stage_ = Stage::WaitServer;
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    const double t = seconds();
    switch (stage_) {
      case Stage::WaitServer:
        if (client_->action_server_is_ready()) {
          send();
        } else if (t > skills_->server_wait) {
          return finish(false, SkillError::NOT_READY, "skill server is not available", nullptr);
        }
        return BT::NodeStatus::RUNNING;

      case Stage::Retry:
        if (t >= retry_at_) {
          send();
        }
        return BT::NodeStatus::RUNNING;

      case Stage::Sending:
        if (goal_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
          if (t > skills_->timeout(skill_)) {
            return finish(false, SkillError::TIMEOUT, "no response to the goal", nullptr);
          }
          return BT::NodeStatus::RUNNING;
        }
        handle_ = goal_future_.get();
        if (!handle_) {
          if (t < skills_->busy_retry) {  // the skill server rejects goals while it is busy
            stage_ = Stage::Retry;
            retry_at_ = t + 0.25;
            return BT::NodeStatus::RUNNING;
          }
          return finish(false, SkillError::NOT_READY, "goal rejected (skill server busy?)", nullptr);
        }
        result_future_ = client_->async_get_result(handle_);
        stage_ = Stage::Running;
        return BT::NodeStatus::RUNNING;

      case Stage::Running:
        // check the result before the timeout, since after a pause it may already be here
        if (result_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
          const auto wrapped = result_future_.get();
          handle_.reset();
          if (!wrapped.result) {
            return finish(false, SkillError::EXECUTION_FAILED, "no result", nullptr);
          }
          const Result & r = *wrapped.result;
          // At startup the action servers exist before the skill server is ready, because MoveIt
          // and the controllers come up later, and later still with the GUIs up. The executor
          // first failed the whole task in 0.1 s on that NOT_READY, so the node now waits and
          // sends again, for up to ready_wait.
          if (!r.success && r.error.code == SkillError::NOT_READY &&
            waited_ + t < skills_->ready_wait)
          {
            if (waited_ == 0.0) {
              ctx_->info(skill_ + ": skill server is still starting up; waiting");
            }
            waited_ += t;
            start_ = std::chrono::steady_clock::now();
            retry_at_ = 1.0;
            stage_ = Stage::Retry;
            return BT::NodeStatus::RUNNING;
          }
          const bool ok = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && r.success;
          const uint8_t code = ok ? SkillError::SUCCESS :
            (r.error.code != SkillError::SUCCESS ? r.error.code :
            wrapped.code == rclcpp_action::ResultCode::CANCELED ? SkillError::CANCELED :
            SkillError::EXECUTION_FAILED);
          return finish(ok, code, r.message, &r);
        }
        if (t > skills_->timeout(skill_)) {
          cancel();
          return finish(false, SkillError::TIMEOUT,
                   "timed out after " + std::to_string(static_cast<int>(t)) + " s", nullptr);
        }
        return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    // The skill may have finished while the tree was not being ticked, during a pause. Abort
    // while paused used to crash exactly there: cancelling that forgotten goal threw
    // UnknownGoalHandleError. So a finished skill has its real outcome recorded instead.
    if (stage_ == Stage::Running && handle_ &&
      result_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      onRunning();
      return;
    }
    if (stage_ == Stage::Sending && goal_future_.valid() &&
      goal_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      handle_ = goal_future_.get();  // accepted just now, cancel it below
    }
    cancel();
    finish(false, SkillError::CANCELED, "halted", nullptr);
  }

  // Cancels the goal if there is one. It never throws, because the goal may already have
  // finished on the server.
  void cancel()
  {
    if (!handle_) {
      return;
    }
    try {
      client_->async_cancel_goal(handle_);
    } catch (const std::exception & e) {
      ctx_->warn(skill_ + ": could not cancel the goal: " + e.what());
    }
    handle_.reset();
  }

  void send()
  {
    typename Client::SendGoalOptions options;
    if constexpr (detail::HasPhase<Feedback>::value) {
      auto ctx = ctx_;
      options.feedback_callback =
        [ctx](typename GoalHandle::SharedPtr, const std::shared_ptr<const Feedback> feedback) {
          ctx->updateProgress([&](Progress & p) {p.phase = feedback->phase;});
        };
    }
    goal_future_ = client_->async_send_goal(goal_, options);
    stage_ = Stage::Sending;
  }

  BT::NodeStatus finish(bool ok, uint8_t code, const std::string & message, const Result * result)
  {
    const double duration = std::round(seconds() * 100.0) / 100.0;
    Json record = {
      {"event", "skill"}, {"skill", skill_}, {"target", target_},
      {"result", ok ? "SUCCESS" : "FAILURE"},
      {"error", ok ? Json(nullptr) : Json(errorName(code))},
      {"message", message}, {"duration_s", duration}};
    onResult(ok, result, record);
    ctx_->log(record);
    const std::string holding = ctx_->world.held.value_or("");
    ctx_->updateProgress([&](Progress & p) {
        p.skill.clear();
        p.phase.clear();
        p.holding = holding;
      });
    if (ok) {
      ctx_->info(skill_ + " ok (" + std::to_string(duration).substr(0, 5) + " s)");
      return BT::NodeStatus::SUCCESS;
    }
    ++ctx_->skill_failures;
    ctx_->last_error = skill_ + ": " + errorName(code) + (message.empty() ? "" : ", " + message);
    ctx_->warn(ctx_->last_error);
    return BT::NodeStatus::FAILURE;
  }

  double seconds() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
  }

  typename Client::SharedPtr client_;
  std::string skill_;
  Goal goal_;
  Stage stage_ = Stage::WaitServer;
  std::chrono::steady_clock::time_point start_;
  double retry_at_ = 0.0;
  double waited_ = 0.0;  // s spent waiting for the skill server to become ready
  std::shared_future<typename GoalHandle::SharedPtr> goal_future_;
  typename GoalHandle::SharedPtr handle_;
  std::shared_future<typename GoalHandle::WrappedResult> result_future_;
};

// The skills

// Detects objects and updates the world model. Ports: class and color, both optional filters.
class DetectObjectsNode : public SkillNode<cell_interfaces::action::DetectObjects>
{
public:
  DetectObjectsNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx, std::shared_ptr<SkillClients> skills);
  static BT::PortsList providedPorts();

private:
  std::optional<std::string> makeGoal(Goal & goal) override;
  void onResult(bool success, const Result * result, Json & record) override;
  ObjectFilter filter_;
};

// Picks a detected object. Ports: object (an id or alias, may use $item) and grasp (the
// strategy; empty means by object class, from the executor's grasp_strategies).
class PickNode : public SkillNode<cell_interfaces::action::Pick>
{
public:
  PickNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx, std::shared_ptr<SkillClients> skills);
  static BT::PortsList providedPorts();

private:
  std::optional<std::string> makeGoal(Goal & goal) override;
  void onResult(bool success, const Result * result, Json & record) override;
  std::string object_;
};

// Places the held object. Ports: target (a workspace name, may use $index and $item; a bin gets
// its next drop spot) and compliance (true, false or default).
class PlaceNode : public SkillNode<cell_interfaces::action::Place>
{
public:
  PlaceNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx, std::shared_ptr<SkillClients> skills);
  static BT::PortsList providedPorts();

private:
  std::optional<std::string> makeGoal(Goal & goal) override;
  void onResult(bool success, const Result * result, Json & record) override;
};

// Inserts the held peg. Ports: target (the fixture), max_force and max_attempts (0 for either
// means the skill's default).
class InsertNode : public SkillNode<cell_interfaces::action::Insert>
{
public:
  InsertNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx, std::shared_ptr<SkillClients> skills);
  static BT::PortsList providedPorts();

private:
  std::optional<std::string> makeGoal(Goal & goal) override;
  void onResult(bool success, const Result * result, Json & record) override;
};

// Moves to a named pose from the SRDF. Port: pose.
class MoveToNamedPoseNode : public SkillNode<cell_interfaces::action::MoveToNamedPose>
{
public:
  MoveToNamedPoseNode(const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<RunContext> ctx, std::shared_ptr<SkillClients> skills);
  static BT::PortsList providedPorts();

private:
  std::optional<std::string> makeGoal(Goal & goal) override;
};

// registers the five skill nodes with the factory
void registerSkillNodes(
  BT::BehaviorTreeFactory & factory, std::shared_ptr<RunContext> ctx,
  std::shared_ptr<SkillClients> skills);

}  // namespace cell_executor
