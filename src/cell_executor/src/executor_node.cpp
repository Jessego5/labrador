// This is the task executor node. It loads a task YAML, turns it into a behavior tree and runs
// that tree against the cell_skills action servers, one task at a time, on a worker thread of its
// own so the ROS executor stays free to deliver the skills' responses, the pause, resume and abort
// calls and the state timer. What it offers:
//
//   action   ~/execute_task  cell_interfaces/ExecuteTask   (one task at a time)
//   services ~/pause ~/resume ~/abort                       std_srvs/Trigger
//   topic    ~/state                                        std_msgs/String (JSON)
//   Groot2   live view on tcp port `groot_port` (default 1667)
//
// With the `task` parameter set, the task runs once at startup; run it that way with
// `ros2 launch cell_bringup cell.launch.py task:=sort_and_insert`. Every run writes
// runs/task_<name>_<time>/ with the task, the generated tree, a JSON-lines log of every step and
// skill call, a Groot2 .btlog and a summary.
#include <atomic>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_file_logger_v2.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <cell_interfaces/action/execute_task.hpp>
#include <cell_workspace/workspace.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cell_executor/control_nodes.hpp"
#include "cell_executor/run_context.hpp"
#include "cell_executor/skill_nodes.hpp"
#include "cell_executor/task.hpp"

namespace fs = std::filesystem;

namespace cell_executor
{

using ExecuteTask = cell_interfaces::action::ExecuteTask;
using GoalHandleExecute = rclcpp_action::ServerGoalHandle<ExecuteTask>;
using Trigger = std_srvs::srv::Trigger;

class TaskExecutor : public rclcpp::Node
{
public:
  explicit TaskExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("executor", options)
  {
    const std::string share = ament_index_cpp::get_package_share_directory("cell_executor");
    const std::string description =
      ament_index_cpp::get_package_share_directory("cell_description");
    const char * home = std::getenv("HOME");

    tasks_dir_ = declare_parameter<std::string>("tasks_dir", share + "/tasks");
    trees_dir_ = declare_parameter<std::string>("trees_dir", share + "/trees");
    log_dir_ = declare_parameter<std::string>(
      "log_dir", std::string(home ? home : "/tmp") + "/cell_ws/runs");
    const std::string workspace_file = declare_parameter<std::string>(
      "workspace_file", description + "/config/workspace.yaml");
    groot_port_ = static_cast<int>(declare_parameter<int64_t>("groot_port", 1667));
    tick_period_ = std::chrono::duration<double>(
      1.0 / declare_parameter<double>("tick_rate_hz", 50.0));
    place_compliance_ = declare_parameter<bool>("place_compliance", true);
    for (const auto & entry : declare_parameter<std::vector<std::string>>(
        "grasp_strategies", std::vector<std::string>{"peg:insert"}))
    {
      const auto colon = entry.find(':');
      if (colon == std::string::npos) {
        throw std::invalid_argument("grasp_strategies: expected 'class:strategy', got " + entry);
      }
      grasp_strategies_[entry.substr(0, colon)] = entry.substr(colon + 1);
    }

    skills_ = std::make_shared<SkillClients>();
    skills_->server_wait = declare_parameter<double>("server_wait", 10.0);
    skills_->busy_retry = declare_parameter<double>("busy_retry", 5.0);
    skills_->ready_wait = declare_parameter<double>("ready_wait", 180.0);
    for (const auto & [skill, seconds] : std::vector<std::pair<std::string, double>>{
        {"DetectObjects", 60.0}, {"Pick", 90.0}, {"Place", 90.0}, {"Insert", 300.0},
        {"MoveToNamedPose", 60.0}})
    {
      skills_->timeouts[skill] = declare_parameter<double>("timeouts." + skill, seconds);
    }

    workspace_ = cell_workspace::loadWorkspaceFile(workspace_file);
    const auto problems = workspace_.validate();
    if (!problems.empty()) {
      throw std::runtime_error("invalid workspace " + workspace_file + ": " + problems.front());
    }

    clients_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    skills_->detect = rclcpp_action::create_client<cell_interfaces::action::DetectObjects>(
      this, "/skills/detect_objects", clients_group_);
    skills_->pick = rclcpp_action::create_client<cell_interfaces::action::Pick>(
      this, "/skills/pick", clients_group_);
    skills_->place = rclcpp_action::create_client<cell_interfaces::action::Place>(
      this, "/skills/place", clients_group_);
    skills_->insert = rclcpp_action::create_client<cell_interfaces::action::Insert>(
      this, "/skills/insert", clients_group_);
    skills_->move = rclcpp_action::create_client<cell_interfaces::action::MoveToNamedPose>(
      this, "/skills/move_to_named_pose", clients_group_);

    state_pub_ = create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS(10).reliable());
    state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / declare_parameter<double>("state_rate_hz", 2.0)),
      [this]() {publishState();});

    server_ = rclcpp_action::create_server<ExecuteTask>(
      this, "~/execute_task",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteTask::Goal>) {
        return busy_.load() ? rclcpp_action::GoalResponse::REJECT :
        rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<GoalHandleExecute>) {
        abort_ = true;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandleExecute> handle) {
        startRun(handle->get_goal()->task_file, handle->get_goal()->task_yaml, handle);
      });

    pause_srv_ = create_service<Trigger>(
      "~/pause", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        res->success = busy_.load() && !paused_.exchange(true);
        res->message = res->success ? "pausing: the running skill finishes, nothing new starts" :
        busy_.load() ? "already paused" : "no task running";
        RCLCPP_INFO(get_logger(), "pause: %s", res->message.c_str());
      });
    resume_srv_ = create_service<Trigger>(
      "~/resume", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        res->success = busy_.load() && paused_.exchange(false);
        res->message = res->success ? "resumed" : busy_.load() ? "not paused" : "no task running";
        RCLCPP_INFO(get_logger(), "resume: %s", res->message.c_str());
      });
    abort_srv_ = create_service<Trigger>(
      "~/abort", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        res->success = busy_.load();
        abort_ = res->success;
        res->message = res->success ? "aborting: cancelling the running skill" : "no task running";
        RCLCPP_INFO(get_logger(), "abort: %s", res->message.c_str());
      });

    const std::string task = declare_parameter<std::string>("task", "");
    if (!task.empty() && declare_parameter<bool>("autostart", true)) {
      autostart_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this, task]() {
            autostart_timer_->cancel();
            startRun(task, "", nullptr);
          });
    }
    RCLCPP_INFO(get_logger(), "Task executor ready (tasks in %s, Groot2 port %d)",
      tasks_dir_.c_str(), groot_port_);
  }

  ~TaskExecutor() override
  {
    abort_ = true;
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void startRun(
    const std::string & task_file, const std::string & task_yaml,
    std::shared_ptr<GoalHandleExecute> handle)
  {
    if (busy_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "a task is already running");
      return;
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    abort_ = false;
    paused_ = false;
    worker_ = std::thread([this, task_file, task_yaml, handle]() {
          auto result = std::make_shared<ExecuteTask::Result>();
          try {
            run(task_file, task_yaml, handle, *result);
          } catch (const std::exception & e) {
            result->success = false;
            result->outcome = "invalid";
            result->message = e.what();
          }
          RCLCPP_INFO(get_logger(), "Task %s: %s", result->outcome.c_str(),
          result->message.c_str());
          {
            std::lock_guard<std::mutex> lock(mutex_);
            last_outcome_ = result->outcome;
            last_run_dir_ = result->run_dir;
            ctx_.reset();
          }
          busy_ = false;
          paused_ = false;
          publishState();
          if (handle) {
            if (abort_ && handle->is_canceling()) {
              handle->canceled(result);
            } else if (result->success) {
              handle->succeed(result);
            } else {
              handle->abort(result);
            }
          }
        });
  }

  std::string resolveTaskPath(const std::string & task) const
  {
    if (task.find('/') == std::string::npos) {
      const fs::path named = fs::path(tasks_dir_) /
        (fs::path(task).extension() == ".yaml" ? task : task + ".yaml");
      if (fs::exists(named)) {
        return named.string();
      }
    }
    return task;
  }

  static std::string timestamp()
  {
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return ss.str();
  }

  // Checks every target that does not depend on a loop variable, before the robot moves, so a
  // typo in a target name fails the task at once instead of halfway through it.
  void checkTargets(const std::vector<Step> & steps) const
  {
    for (const auto & step : steps) {
      checkTargets(step.body);
      const auto it = step.args.find("target");
      if (it == step.args.end() || !variablesIn(it->second).empty()) {
        continue;
      }
      try {
        workspace_.placePose(it->second);
      } catch (const std::exception & e) {
        throw TaskError("line " + std::to_string(step.line) + ": " + e.what());
      }
    }
  }

  void run(
    const std::string & task_file, const std::string & task_yaml,
    const std::shared_ptr<GoalHandleExecute> & handle, ExecuteTask::Result & result)
  {
    // 1. load and check the task
    Task task;
    std::string source;
    if (!task_file.empty()) {
      source = resolveTaskPath(task_file);
      task = loadTaskFile(source);
    } else if (!task_yaml.empty()) {
      task = parseTask(task_yaml);
      source = "(inline)";
    } else {
      throw TaskError("no task given (set task_file or task_yaml)");
    }
    checkTargets(task.steps);
    const std::string xml = toBehaviorTreeXml(task);

    // 2. the run directory, with the task, the generated tree and the log
    const fs::path run_dir = fs::path(log_dir_) / ("task_" + task.name + "_" + timestamp());
    fs::create_directories(run_dir);
    {
      std::ofstream(run_dir / "tree.xml") << xml;
      std::ofstream out(run_dir / "task.yaml");
      if (!task_file.empty()) {
        std::ifstream in(source);
        out << in.rdbuf();
      } else {
        out << task_yaml;
      }
    }
    result.run_dir = run_dir.string();
    result.steps_total = static_cast<uint32_t>(task.steps.size());

    // 3. build the tree
    auto ctx = std::make_shared<RunContext>();
    ctx->workspace = &workspace_;
    ctx->grasp_strategies = grasp_strategies_;
    ctx->place_compliance = place_compliance_;
    ctx->sink = [this](const std::string & level, const std::string & message) {
        if (level == "error") {
          RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        } else if (level == "warn") {
          RCLCPP_WARN(get_logger(), "%s", message.c_str());
        } else {
          RCLCPP_INFO(get_logger(), "%s", message.c_str());
        }
      };
    ctx->openLog((run_dir / "log.jsonl").string());
    ctx->updateProgress([&](Progress & p) {
        p.state = "running";
        p.task = task.name;
        p.steps_total = static_cast<int>(task.steps.size());
        p.run_dir = run_dir.string();
      });
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ctx_ = ctx;
    }

    BT::BehaviorTreeFactory factory;
    registerControlNodes(factory, ctx);
    registerSkillNodes(factory, ctx, skills_);
    // Files starting with '.' are skipped. AppleDouble `._*.xml` files that came along in a copy
    // from macOS broke tree loading.
    for (const auto & entry : fs::directory_iterator(trees_dir_)) {
      const std::string file = entry.path().filename().string();
      if (entry.path().extension() == ".xml" && file.front() != '.') {
        factory.registerBehaviorTreeFromFile(entry.path().string());
      }
    }
    factory.registerBehaviorTreeFromText(xml);
    BT::Tree tree = factory.createTree("Task");

    std::unique_ptr<BT::Groot2Publisher> groot;
    try {
      groot = std::make_unique<BT::Groot2Publisher>(tree, groot_port_);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Groot2 publisher unavailable on port %d: %s", groot_port_,
        e.what());
    }
    BT::FileLogger2 btlog(tree, run_dir / "tree.btlog");

    ctx->log({{"event", "start"}, {"task", task.name}, {"source", source},
        {"steps", task.steps.size()}, {"on_failure", toString(task.on_failure)}});
    RCLCPP_INFO(get_logger(), "Running task '%s' (%zu steps, on_failure %s) -> %s",
      task.name.c_str(), task.steps.size(), toString(task.on_failure).c_str(),
      run_dir.c_str());

    // 4. tick until done, honouring pause and abort
    const auto t0 = std::chrono::steady_clock::now();
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    bool aborted = false;
    auto last_feedback = t0;
    while (rclcpp::ok()) {
      if (abort_) {
        tree.haltTree();
        aborted = true;
        break;
      }
      const bool paused = paused_.load();
      ctx->updateProgress([&](Progress & p) {p.state = paused ? "paused" : "running";});
      if (!paused) {
        status = tree.tickOnce();
        if (status != BT::NodeStatus::RUNNING) {
          break;
        }
      }
      const auto now = std::chrono::steady_clock::now();
      if (handle && now - last_feedback > std::chrono::milliseconds(500)) {
        last_feedback = now;
        const Progress p = ctx->progress();
        auto feedback = std::make_shared<ExecuteTask::Feedback>();
        feedback->state = p.state;
        feedback->step = static_cast<uint32_t>(p.step);
        feedback->step_name = p.step_name;
        feedback->item = p.item;
        feedback->skill = p.skill;
        feedback->phase = p.phase;
        handle->publish_feedback(feedback);
      }
      tree.sleep(std::chrono::duration_cast<std::chrono::milliseconds>(tick_period_));
    }
    if (!rclcpp::ok() && !aborted && status == BT::NodeStatus::RUNNING) {
      tree.haltTree();
      aborted = true;
    }

    // 5. the outcome and the summary
    const double duration =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    result.success = !aborted && status == BT::NodeStatus::SUCCESS;
    result.outcome = aborted ? "aborted" : result.success ? "succeeded" :
      ctx->safe_stopped ? "safe_stopped" : "failed";
    result.steps_succeeded = static_cast<uint32_t>(ctx->steps_succeeded);
    result.steps_skipped = static_cast<uint32_t>(ctx->steps_skipped);
    result.skill_calls = static_cast<uint32_t>(ctx->skill_calls);
    result.skill_failures = static_cast<uint32_t>(ctx->skill_failures);
    result.duration_s = static_cast<float>(duration);
    std::ostringstream message;
    message << task.name << ": " << result.steps_succeeded << "/" << result.steps_total
            << " steps succeeded";
    if (ctx->steps_skipped || ctx->items_skipped) {
      message << ", " << ctx->steps_skipped << " step(s) and " << ctx->items_skipped
              << " object(s) skipped";
    }
    if (ctx->failed_step) {
      message << "; stopped at step " << ctx->failed_step;
      if (!ctx->last_error.empty()) {
        message << " (" << ctx->last_error << ")";
      }
    }
    message << " in " << std::fixed << std::setprecision(1) << duration << " s";
    result.message = message.str();

    const Json summary = {
      {"task", task.name}, {"source", source}, {"outcome", result.outcome},
      {"success", result.success}, {"message", result.message},
      {"steps_total", result.steps_total}, {"steps_succeeded", result.steps_succeeded},
      {"steps_skipped", result.steps_skipped}, {"objects_skipped", ctx->items_skipped},
      {"skill_calls", result.skill_calls}, {"skill_failures", result.skill_failures},
      {"failed_step", ctx->failed_step}, {"duration_s", std::round(duration * 100.0) / 100.0}};
    ctx->log({{"event", "end"}, {"outcome", result.outcome}, {"message", result.message}});
    std::ofstream(run_dir / "summary.json") << summary.dump(2) << "\n";
  }

  void publishState()
  {
    Json state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (ctx_) {
        const Progress p = ctx_->progress();
        state = {
          {"state", p.state}, {"task", p.task}, {"step", p.step},
          {"steps_total", p.steps_total}, {"step_name", p.step_name}, {"item", p.item},
          {"skill", p.skill}, {"phase", p.phase}, {"run_dir", p.run_dir},
          {"elapsed_s", std::round(ctx_->elapsed() * 10.0) / 10.0},
          {"holding", p.holding}};
      } else {
        state = {{"state", "idle"}, {"last_outcome", last_outcome_},
          {"last_run_dir", last_run_dir_}};
      }
    }
    std_msgs::msg::String msg;
    msg.data = state.dump();
    state_pub_->publish(msg);
  }

  std::string tasks_dir_, trees_dir_, log_dir_;
  int groot_port_ = 1667;
  std::chrono::duration<double> tick_period_{0.02};
  bool place_compliance_ = true;
  std::map<std::string, std::string> grasp_strategies_;
  cell_workspace::Workspace workspace_;

  std::shared_ptr<SkillClients> skills_;
  rclcpp::CallbackGroup::SharedPtr clients_group_;
  rclcpp_action::Server<ExecuteTask>::SharedPtr server_;
  rclcpp::Service<Trigger>::SharedPtr pause_srv_, resume_srv_, abort_srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_, autostart_timer_;

  std::atomic<bool> busy_{false}, paused_{false}, abort_{false};
  std::thread worker_;
  std::mutex mutex_;
  std::shared_ptr<RunContext> ctx_;  // the running task's context, for the state topic
  std::string last_outcome_, last_run_dir_;
};

}  // namespace cell_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cell_executor::TaskExecutor>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
