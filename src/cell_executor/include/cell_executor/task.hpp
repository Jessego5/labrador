// This is the task file format, a YAML list of steps (detect, pick, place, insert, move_to,
// dispense, wait, for_each, watch, if, repeat_until) with a failure policy and the number of
// recoveries each pick, place and insert gets, parsed into a Task and then turned into the
// BehaviorTree.CPP XML that the executor runs. It has no ROS in it, so the parser, the validation
// messages and the generated XML are unit tested in milliseconds by test_task, and the executor
// node only has to load the result. The format itself is described in cell_executor/README.md.
#pragma once

#include <functional>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace cell_executor
{

// What to do when a step fails, after the skills' own retries have been used up.
enum class FailurePolicy
{
  SafeStop,  // put down what the gripper holds, go home, end the task as failed
  SkipStep,  // log it and carry on with the next step (in a for_each, the next object)
  Abort,     // stop the task where it is
};

// "safe_stop", "skip_step" or "abort"
std::string toString(FailurePolicy policy);
// the policy named by `text`. Throws std::invalid_argument for anything else.
FailurePolicy parseFailurePolicy(const std::string & text);

// an object filter, field to required value, over the fields "class", "color" and "type"
using ObjectFilter = std::map<std::string, std::string>;

// "class=cylinder;color=blue" (sorted by field), the form stored in an XML attribute
std::string encodeFilter(const ObjectFilter & filter);
// The filter back from encodeFilter's text. Throws std::invalid_argument on malformed text or an
// unknown field.
ObjectFilter decodeFilter(const std::string & text);

// A condition on what an object looks like now: field to the values that satisfy it, e.g.
// {"color": {"green", "brown"}}. The fields are the filter fields.
using ObjectCondition = std::map<std::string, std::vector<std::string>>;

// "color=brown,green" (fields sorted, values sorted), the form stored in an XML attribute
std::string encodeCondition(const ObjectCondition & condition);
// The condition back from encodeCondition's text. Throws std::invalid_argument when malformed.
ObjectCondition decodeCondition(const std::string & text);

// one task step
struct Step
{
  // detect, pick, place, insert, move_to, dispense, wait, for_each, watch, if, repeat_until
  std::string kind;
  std::map<std::string, std::string> args;  // scalar arguments, as written
  ObjectFilter filter;                      // for_each and watch: which objects (empty = all)
  ObjectCondition condition;                // watch: when to act; if, repeat_until: the test
  int max = 0;                              // for_each: at most this many objects (0 = all);
                                            // repeat_until: at most this many rounds
  std::vector<Step> body;                   // for_each, watch, repeat_until: do; if: then
  std::vector<Step> else_body;              // if: else (may be empty)
  int line = 0;                             // 1-based line in the task file

  // a short human-readable form, e.g. "place target=tray_1/C$index"
  std::string describe() const;
};

struct Task
{
  std::string name;
  std::string description;
  FailurePolicy on_failure = FailurePolicy::SafeStop;
  int retries = 0;  // recoveries per pick, place and insert unless the step sets its own
  std::vector<Step> steps;
};

// A malformed task file. The message starts with "line N:" when the location is known, so the
// person editing the file can go straight to it.
class TaskError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

// The parsed and validated task. Throws TaskError.
Task parseTask(const std::string & yaml);
// The task in the file at `path`. Throws TaskError, also when the file cannot be read.
Task loadTaskFile(const std::string & path);

// The main tree ("Task") for a task. Each top-level step is wrapped in a `Step` decorator that
// applies the failure policy, and with on_failure = safe_stop the task falls back to the
// `SafeStop` subtree (trees/safe_stop.xml), which has to be registered with the factory as well.
std::string toBehaviorTreeXml(const Task & task);

// the names of the variables ($item, $index, ...) used in `text`, in order of appearance
std::vector<std::string> variablesIn(const std::string & text);

// `text` with every $name replaced by lookup(name). A '$' not followed by a name is kept. Throws
// std::runtime_error when lookup returns nothing for a variable.
std::string expandVariables(
  const std::string & text,
  const std::function<std::optional<std::string>(const std::string &)> & lookup);

}  // namespace cell_executor
