// This is the implementation of the task-structure nodes declared in control_nodes.hpp. Step,
// ForEachObject and WatchObjects write a log record for each finished step and each skipped object,
// and keep /executor/state's step and item up to date through the RunContext. Recover logs each
// recovery it starts and whether the skill then succeeded, which is what the reliability benchmark
// scores the injected faults against.
#include "cell_executor/control_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace cell_executor
{

namespace
{

double rounded(double seconds)
{
  return std::round(seconds * 100.0) / 100.0;
}

// the object an "item" port names, with $item and $index filled in from the blackboard
std::string itemOf(const BT::TreeNode & node)
{
  const std::string text = node.getInput<std::string>("item").value_or("$item");
  return expandVariables(text, [&](const std::string & name) -> std::optional<std::string> {
             std::string value;
             if (node.config().blackboard->get(name, value)) {
               return value;
             }
             return std::nullopt;
           });
}

std::string describeCondition(const ObjectCondition & condition)
{
  return encodeCondition(condition);
}

}  // namespace

// Step

StepNode::StepNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx)
: BT::DecoratorNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList StepNode::providedPorts()
{
  return {
    BT::InputPort<int>("index", "1-based step number"),
    BT::InputPort<std::string>("title", "", "what the step does"),
    BT::InputPort<std::string>("on_failure", "safe_stop", "safe_stop, skip_step or abort"),
  };
}

BT::NodeStatus StepNode::tick()
{
  const int index = getInput<int>("index").value_or(0);
  const std::string title = getInput<std::string>("title").value_or("");
  if (status() == BT::NodeStatus::IDLE) {
    started_ = ctx_->elapsed();
    ctx_->last_error.clear();
    ctx_->updateProgress([&](Progress & p) {
        p.step = index;
        p.step_name = title;
        p.item.clear();
        p.skill.clear();
        p.phase.clear();
      });
    ctx_->info("step " + std::to_string(index) + ": " + title);
  }
  setStatus(BT::NodeStatus::RUNNING);

  const BT::NodeStatus child = child_node_->executeTick();
  if (child == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  resetChild();
  const double duration = rounded(ctx_->elapsed() - started_);
  if (child == BT::NodeStatus::SUCCESS || child == BT::NodeStatus::SKIPPED) {
    ++ctx_->steps_succeeded;
    ctx_->log({{"event", "step"}, {"step", index}, {"title", title}, {"result", "SUCCESS"},
        {"duration_s", duration}});
    return BT::NodeStatus::SUCCESS;
  }

  FailurePolicy policy = FailurePolicy::SafeStop;
  try {
    policy = parseFailurePolicy(getInput<std::string>("on_failure").value_or("safe_stop"));
  } catch (const std::invalid_argument & e) {
    ctx_->error(e.what());
  }
  ctx_->log({{"event", "step"}, {"step", index}, {"title", title}, {"result", "FAILURE"},
      {"error", ctx_->last_error}, {"policy", toString(policy)}, {"duration_s", duration}});
  const std::string why = ctx_->last_error.empty() ? "" : " (" + ctx_->last_error + ")";
  if (policy == FailurePolicy::SkipStep) {
    ++ctx_->steps_skipped;
    ctx_->warn("step " + std::to_string(index) + " failed" + why + "; skipping it");
    return BT::NodeStatus::SUCCESS;
  }
  ctx_->failed_step = index;
  ctx_->safe_stopped = policy == FailurePolicy::SafeStop;
  ctx_->error("step " + std::to_string(index) + " failed" + why +
    (ctx_->safe_stopped ? "; safe stop" : "; aborting"));
  return BT::NodeStatus::FAILURE;
}

// ForEachObject

ForEachObjectNode::ForEachObjectNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx)
: BT::DecoratorNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList ForEachObjectNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("filter", "", "field=value;... with fields class, color, type"),
    BT::InputPort<int>("max", 0, "at most this many objects (0 = all)"),
    BT::InputPort<std::string>("on_failure", "safe_stop", "skip_step skips a failed object"),
  };
}

BT::NodeStatus ForEachObjectNode::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    try {
      filter_ = decodeFilter(getInput<std::string>("filter").value_or(""));
    } catch (const std::invalid_argument & e) {
      ctx_->last_error = std::string("for_each: ") + e.what();
      return BT::NodeStatus::FAILURE;
    }
    max_ = getInput<int>("max").value_or(0);
    items_ = ctx_->world.select(filter_, max_);
    current_ = 0;
    item_started_ = false;
    if (items_.empty()) {
      ctx_->warn("for_each: no detected objects match '" + encodeFilter(filter_) + "'");
      return BT::NodeStatus::SUCCESS;
    }
    std::string list;
    for (const auto & id : items_) {
      list += (list.empty() ? "" : ", ") + id;
    }
    ctx_->info("for_each: " + std::to_string(items_.size()) + " object(s): " + list);
  }
  setStatus(BT::NodeStatus::RUNNING);

  if (!item_started_) {
    const std::string & id = items_[current_];
    config().blackboard->set("item", id);
    config().blackboard->set("index", std::to_string(current_ + 1));
    ctx_->last_error.clear();
    ctx_->updateProgress([&](Progress & p) {p.item = id;});
    item_started_ = true;
  }

  const BT::NodeStatus child = child_node_->executeTick();
  if (child == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  resetChild();
  item_started_ = false;
  if (child == BT::NodeStatus::FAILURE) {
    const bool skip = getInput<std::string>("on_failure").value_or("") == "skip_step";
    if (!skip) {
      return BT::NodeStatus::FAILURE;
    }
    ++ctx_->items_skipped;
    ctx_->warn("for_each: skipping " + items_[current_] +
      (ctx_->last_error.empty() ? "" : " (" + ctx_->last_error + ")"));
    ctx_->log({{"event", "item_skipped"}, {"item", items_[current_]},
        {"error", ctx_->last_error}});
  }
  if (++current_ >= items_.size() && !addNewItems()) {
    ctx_->updateProgress([](Progress & p) {p.item.clear();});
    return BT::NodeStatus::SUCCESS;
  }
  // go back to the tick loop between objects, so a pause or abort takes effect at a clean point
  return BT::NodeStatus::RUNNING;
}

bool ForEachObjectNode::addNewItems()
{
  std::string list;
  for (const auto & id : ctx_->world.select(filter_, 0)) {
    if (max_ > 0 && static_cast<int>(items_.size()) >= max_) {
      break;
    }
    if (std::find(items_.begin(), items_.end(), id) == items_.end()) {
      items_.push_back(id);
      list += (list.empty() ? "" : ", ") + id;
    }
  }
  if (!list.empty()) {
    ctx_->info("for_each: seen since the loop started: " + list);
  }
  return current_ < items_.size();
}

void ForEachObjectNode::halt()
{
  items_.clear();
  current_ = 0;
  item_started_ = false;
  BT::DecoratorNode::halt();
}

// Recover

RecoverNode::RecoverNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx)
: BT::ControlNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList RecoverNode::providedPorts()
{
  return {
    BT::InputPort<int>("attempts", 3, "tries of the skill in all, recoveries in between"),
    BT::InputPort<std::string>("what", "", "the step kind, for the log"),
  };
}

bool RecoverNode::retryable(const std::string & error)
{
  static const std::set<std::string> kRetryable = {
    "PLAN_FAILED", "EXECUTION_FAILED", "GRASP_FAILED", "RELEASE_FAILED", "INSERT_JAMMED",
    "TIMEOUT", "NOT_HOLDING", "PERCEPTION_FAILED"};
  return kRetryable.count(error) > 0;
}

BT::NodeStatus RecoverNode::tick()
{
  if (children_nodes_.size() != 2) {
    throw BT::RuntimeError("Recover needs two children: the skill and its recovery");
  }
  const int attempts = std::max(1, getInput<int>("attempts").value_or(3));
  const std::string what = getInput<std::string>("what").value_or("");
  if (status() == BT::NodeStatus::IDLE) {
    attempt_ = 1;
    recovering_ = false;
    config().blackboard->set("recover_object", ctx_->world.held.value_or(""));
  }
  setStatus(BT::NodeStatus::RUNNING);

  if (recovering_) {
    const BT::NodeStatus r = children_nodes_[1]->executeTick();
    if (r == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    resetChildren();
    if (r == BT::NodeStatus::FAILURE && attempt_ + 1 < attempts &&
      retryable(ctx_->last_error_code))
    {
      // The recovery's own pick missed (a drop picked up again can be knocked aside too). That
      // uses up a try, and the recovery runs again; giving up here lost a run with tries left.
      ++attempt_;
      ctx_->warn(what + ": recovery failed (" + ctx_->last_error + "); recovering again (try " +
        std::to_string(attempt_ + 1) + " of " + std::to_string(attempts) + " next)");
      ctx_->log({{"event", "recovery"}, {"what", what}, {"attempt", attempt_},
          {"error", ctx_->last_error_code}, {"message", ctx_->last_error}});
      ctx_->last_error_code.clear();
      return BT::NodeStatus::RUNNING;
    }
    recovering_ = false;
    if (r == BT::NodeStatus::FAILURE) {
      ctx_->last_error = failure_ + "; recovery failed" +
        (ctx_->last_error.empty() || ctx_->last_error == failure_ ? "" : ": " + ctx_->last_error);
      ctx_->log({{"event", "recovery_failed"}, {"what", what}, {"attempt", attempt_},
          {"error", ctx_->last_error}});
      return BT::NodeStatus::FAILURE;
    }
    ++attempt_;
    // back to the tick loop, so a pause or abort takes effect before the skill runs again
    return BT::NodeStatus::RUNNING;
  }

  if (children_nodes_[0]->status() == BT::NodeStatus::IDLE) {
    ctx_->last_error_code.clear();
  }
  const BT::NodeStatus s = children_nodes_[0]->executeTick();
  if (s == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  resetChildren();
  if (s == BT::NodeStatus::SUCCESS) {
    if (attempt_ > 1) {
      ++ctx_->recovered;
      ctx_->info(what + ": recovered on try " + std::to_string(attempt_));
      ctx_->log({{"event", "recovered"}, {"what", what}, {"attempt", attempt_}});
    }
    return BT::NodeStatus::SUCCESS;
  }
  const std::string code = ctx_->last_error_code;
  if (attempt_ >= attempts || !retryable(code)) {
    return BT::NodeStatus::FAILURE;
  }
  if (attempt_ == 1) {
    ++ctx_->recoveries;
  }
  failure_ = ctx_->last_error;
  ctx_->warn(what + ": " + failure_ + "; recovering (try " + std::to_string(attempt_ + 1) +
    " of " + std::to_string(attempts) + " next)");
  ctx_->log({{"event", "recovery"}, {"what", what}, {"attempt", attempt_}, {"error", code},
      {"message", failure_}});
  recovering_ = true;
  return BT::NodeStatus::RUNNING;
}

void RecoverNode::halt()
{
  attempt_ = 1;
  recovering_ = false;
  BT::ControlNode::halt();
}

// IsHolding

IsHoldingNode::IsHoldingNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx)
: BT::ConditionNode(name, config), ctx_(std::move(ctx))
{
}

BT::NodeStatus IsHoldingNode::tick()
{
  return ctx_->world.held ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// WatchObjects

WatchObjectsNode::WatchObjectsNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx)
: BT::ControlNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList WatchObjectsNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("filter", "", "which detected objects to watch"),
    BT::InputPort<std::string>("when", "the condition that makes an object ready, color=a,b"),
    BT::InputPort<double>("period", 3.0, "s between looks while nothing is ready"),
    BT::InputPort<double>("timeout", 300.0, "s before the watch gives up"),
    BT::InputPort<int>("max_failed_looks", 3, "failed looks in a row that end the watch"),
    BT::InputPort<std::string>("on_failure", "safe_stop", "skip_step skips a failed object"),
  };
}

BT::NodeStatus WatchObjectsNode::tick()
{
  if (children_nodes_.size() != 2) {
    throw BT::RuntimeError("WatchObjects needs two children: the look and the per-object steps");
  }
  ObjectFilter filter;
  ObjectCondition when;
  try {
    filter = decodeFilter(getInput<std::string>("filter").value_or(""));
    when = decodeCondition(getInput<std::string>("when").value_or(""));
  } catch (const std::invalid_argument & e) {
    ctx_->last_error = std::string("watch: ") + e.what();
    return BT::NodeStatus::FAILURE;
  }
  const double period = getInput<double>("period").value_or(3.0);
  const double timeout = getInput<double>("timeout").value_or(300.0);
  if (status() == BT::NodeStatus::IDLE) {
    phase_ = Phase::Look;
    watched_.clear();
    done_.clear();
    queue_.clear();
    handled_ = 0;
    failed_looks_ = 0;
    item_started_ = false;
    started_ = ctx_->elapsed();
  }
  setStatus(BT::NodeStatus::RUNNING);

  for (;;) {
    if (phase_ == Phase::Look) {
      const BT::NodeStatus look = children_nodes_[0]->executeTick();
      if (look == BT::NodeStatus::RUNNING) {
        return BT::NodeStatus::RUNNING;
      }
      haltChild(0);
      if (look == BT::NodeStatus::FAILURE) {
        if (++failed_looks_ >= getInput<int>("max_failed_looks").value_or(3)) {
          return BT::NodeStatus::FAILURE;
        }
        ctx_->warn("watch: a look failed (" + ctx_->last_error + "), looking again");
        phase_ = Phase::Wait;
        next_look_ = ctx_->elapsed() + period;
        continue;
      }
      failed_looks_ = 0;
      if (watched_.empty()) {
        watched_ = ctx_->world.select(filter);
        if (watched_.empty()) {
          ctx_->warn("watch: no detected objects match '" + encodeFilter(filter) + "'");
          return BT::NodeStatus::SUCCESS;
        }
        std::string list;
        for (const auto & id : watched_) {
          list += (list.empty() ? "" : ", ") + id;
        }
        ctx_->info("watch: watching " + std::to_string(watched_.size()) + " object(s) until " +
          describeCondition(when) + ": " + list);
      }
      std::size_t open = 0;
      for (const auto & id : watched_) {
        if (done_.count(id) || ctx_->world.handled(id)) {
          continue;
        }
        ++open;
        if (ctx_->world.meets(id, when) &&
          std::find(queue_.begin(), queue_.end(), id) == queue_.end())
        {
          queue_.push_back(id);
        }
      }
      if (open == 0) {
        ctx_->updateProgress([](Progress & p) {p.item.clear();});
        return BT::NodeStatus::SUCCESS;
      }
      if (queue_.empty()) {
        phase_ = Phase::Wait;
        next_look_ = ctx_->elapsed() + period;
      } else {
        phase_ = Phase::Handle;
      }
      continue;
    }

    if (phase_ == Phase::Wait) {
      const double now = ctx_->elapsed();
      if (now - started_ > timeout) {
        std::size_t open = 0;
        for (const auto & id : watched_) {
          open += !done_.count(id) && !ctx_->world.handled(id);
        }
        ctx_->last_error = "watch: timed out after " + std::to_string(static_cast<int>(timeout)) +
          " s with " + std::to_string(open) + " object(s) never meeting " +
          describeCondition(when);
        return BT::NodeStatus::FAILURE;
      }
      if (now < next_look_) {
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::Look;
      continue;
    }

    // Handle the object at the front of the queue.
    const std::string id = queue_.front();
    if (!item_started_) {
      config().blackboard->set("item", id);
      config().blackboard->set("index", std::to_string(++handled_));
      ctx_->last_error.clear();
      ctx_->updateProgress([&](Progress & p) {p.item = id;});
      const auto o = ctx_->world.find(id);
      std::string seen = o ? o->color : std::string("?");
      if (o && o->inner_area > 0.0) {
        seen += ", inner area " + std::to_string(static_cast<int>(1e6 * o->inner_area)) + " mm2";
      }
      ctx_->info("watch: " + id + " is " + seen + ", handling it");
      ctx_->log({{"event", "watch_ready"}, {"item", id}, {"color", o ? o->color : ""},
          {"hue", o ? o->hue : 0.0}, {"inner_area_mm2", o ? 1e6 * o->inner_area : 0.0}});
      item_started_ = true;
    }
    const BT::NodeStatus child = children_nodes_[1]->executeTick();
    if (child == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    haltChild(1);
    item_started_ = false;
    queue_.pop_front();
    done_.insert(id);
    if (child == BT::NodeStatus::FAILURE) {
      if (getInput<std::string>("on_failure").value_or("") != "skip_step") {
        return BT::NodeStatus::FAILURE;
      }
      ++ctx_->items_skipped;
      ctx_->warn("watch: skipping " + id +
        (ctx_->last_error.empty() ? "" : " (" + ctx_->last_error + ")"));
      ctx_->log({{"event", "item_skipped"}, {"item", id}, {"error", ctx_->last_error}});
    }
    // look again before the next one: time has passed and more may be ready
    queue_.clear();
    phase_ = Phase::Look;
    return BT::NodeStatus::RUNNING;
  }
}

void WatchObjectsNode::halt()
{
  phase_ = Phase::Look;
  queue_.clear();
  item_started_ = false;
  BT::ControlNode::halt();
}

// RepeatUntilObject

RepeatUntilObjectNode::RepeatUntilObjectNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx)
: BT::DecoratorNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList RepeatUntilObjectNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("item", "$item", "the object to test"),
    BT::InputPort<std::string>("condition", "what it must meet, color=red"),
    BT::InputPort<int>("max", 10, "rounds before giving up"),
    BT::InputPort<std::string>("on_max", "fail", "fail, or continue after the last round"),
  };
}

BT::NodeStatus RepeatUntilObjectNode::tick()
{
  const std::string id = itemOf(*this);
  ObjectCondition condition;
  try {
    condition = decodeCondition(getInput<std::string>("condition").value_or(""));
  } catch (const std::invalid_argument & e) {
    ctx_->last_error = std::string("repeat_until: ") + e.what();
    return BT::NodeStatus::FAILURE;
  }
  const int max = getInput<int>("max").value_or(10);
  if (status() == BT::NodeStatus::IDLE) {
    rounds_ = 0;
    child_running_ = false;
  }
  setStatus(BT::NodeStatus::RUNNING);

  if (!child_running_) {
    if (ctx_->world.meets(id, condition)) {
      ctx_->info("repeat_until: " + id + " meets " + describeCondition(condition) + " after " +
        std::to_string(rounds_) + " round(s)");
      ctx_->log({{"event", "repeat_until"}, {"item", id}, {"rounds", rounds_},
          {"result", "SUCCESS"}});
      return BT::NodeStatus::SUCCESS;
    }
    if (rounds_ >= max) {
      const auto o = ctx_->world.find(id);
      ctx_->last_error = "repeat_until: " + id + " did not meet " +
        describeCondition(condition) + " after " + std::to_string(max) + " rounds (it is " +
        (o ? o->color : std::string("not detected")) + ")";
      ctx_->log({{"event", "repeat_until"}, {"item", id}, {"rounds", rounds_},
          {"result", "FAILURE"}});
      if (getInput<std::string>("on_max").value_or("fail") == "continue") {
        ctx_->warn(ctx_->last_error + "; carrying on");
        ctx_->last_error.clear();
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
    }
    child_running_ = true;
  }
  const BT::NodeStatus child = child_node_->executeTick();
  if (child == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  resetChild();
  child_running_ = false;
  if (child == BT::NodeStatus::FAILURE) {
    return BT::NodeStatus::FAILURE;
  }
  ++rounds_;
  return BT::NodeStatus::RUNNING;  // check the condition on the next tick
}

void RepeatUntilObjectNode::halt()
{
  rounds_ = 0;
  child_running_ = false;
  BT::DecoratorNode::halt();
}

// ObjectIs

ObjectIsNode::ObjectIsNode(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<RunContext> ctx)
: BT::ConditionNode(name, config), ctx_(std::move(ctx))
{
}

BT::PortsList ObjectIsNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("item", "$item", "the object to test"),
    BT::InputPort<std::string>("condition", "what it must meet, color=green"),
  };
}

BT::NodeStatus ObjectIsNode::tick()
{
  const std::string id = itemOf(*this);
  ObjectCondition condition;
  try {
    condition = decodeCondition(getInput<std::string>("condition").value_or(""));
  } catch (const std::invalid_argument & e) {
    ctx_->error(std::string("if: ") + e.what());
    return BT::NodeStatus::FAILURE;
  }
  if (!ctx_->world.find(id)) {
    ctx_->warn("if: no detected object '" + id + "', taking the else branch");
    return BT::NodeStatus::FAILURE;
  }
  return ctx_->world.meets(id, condition) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void registerControlNodes(BT::BehaviorTreeFactory & factory, std::shared_ptr<RunContext> ctx)
{
  factory.registerNodeType<StepNode>("Step", ctx);
  factory.registerNodeType<ForEachObjectNode>("ForEachObject", ctx);
  factory.registerNodeType<WatchObjectsNode>("WatchObjects", ctx);
  factory.registerNodeType<RepeatUntilObjectNode>("RepeatUntilObject", ctx);
  factory.registerNodeType<ObjectIsNode>("ObjectIs", ctx);
  factory.registerNodeType<IsHoldingNode>("IsHolding", ctx);
  factory.registerNodeType<RecoverNode>("Recover", ctx);
}

}  // namespace cell_executor
