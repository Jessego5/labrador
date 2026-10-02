// This is the state the behavior-tree nodes share during one task run: what the executor knows
// about the scene (World), the live progress behind /executor/state, the counters that go into
// summary.json and the JSON-lines run log. There is no ROS node in it, so the tree logic can be
// tested with stub skills; the executor node fills in the workspace and the message sink.
#pragma once

#include <chrono>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <behaviortree_cpp/contrib/json.hpp>
#include <cell_interfaces/msg/detected_object.hpp>
#include <cell_workspace/workspace.hpp>

#include "cell_executor/task.hpp"

namespace cell_executor
{

using Json = nlohmann::json;
using DetectedObject = cell_interfaces::msg::DetectedObject;

// The scene as the executor sees it: the objects it has detected, which have been put away, and
// what the gripper holds. An object keeps the id it was first detected with for the whole run,
// even when it looks different later. That is what lets a task wait for a vial to change colour:
// the detector names things by what they look like now ("vial_green_0"), so the world matches
// each new detection to the object of the same class it last saw within match_radius and keeps
// that object's id, while updating its colour, type and pose.
class World
{
public:
  // an object whose colour changed between two detections
  struct Change
  {
    std::string id;
    std::string from;
    std::string to;
  };

  // Take in a detection. Each new object is matched to a known one as described above, and the
  // known objects that match `filter` but were not seen again are dropped, unless they are held
  // or have been put away. Returns the colour changes, for the log.
  std::vector<Change> updateDetections(
    const std::vector<DetectedObject> & objects, const ObjectFilter & filter);
  const std::vector<DetectedObject> & detections() const {return objects_;}

  // The object with detection id `ref` ("blue_cylinder_2"), or the one an alias "<class>_<n>" or
  // "<type>_<n>" names: the n-th (0-based) detection of that class or type, ordered by id.
  // "peg_0" is the first detected peg.
  std::optional<DetectedObject> find(const std::string & ref) const;

  // The ids of the detections that match `filter` and have not been put away yet, ordered by id.
  // `max` = 0 means all of them.
  std::vector<std::string> select(const ObjectFilter & filter, int max = 0) const;

  static bool matches(const DetectedObject & object, const ObjectFilter & filter);

  // True when the object `id` meets every field of `condition` now.
  bool meets(const std::string & id, const ObjectCondition & condition) const;

  // Move the object `id` to where it was just placed (the centre of its bounding box), so the
  // next detection matches it there and not as a new object.
  void moveTo(const std::string & id, const Eigen::Vector3d & center);

  // An object is handled from the moment it is placed until it is picked again: loops skip
  // handled objects, and picking one back out of a station (a vial from the dispenser) puts it
  // back in play.
  void markHandled(const std::string & id) {handled_.insert(id);}
  void unmarkHandled(const std::string & id) {handled_.erase(id);}
  bool handled(const std::string & id) const {return handled_.count(id) > 0;}

  double match_radius = 0.02;  // m; vials in a rack are 60 mm apart

  std::optional<std::string> held;  // id of the object in the gripper

private:
  // `id`, or the same name with the lowest free _n suffix when another object already has it
  std::string uniqueId(const std::string & id, const std::set<std::string> & taken) const;

  std::vector<DetectedObject> objects_;
  std::set<std::string> handled_;
};

// the live progress of a run, which is what /executor/state shows
struct Progress
{
  std::string state = "idle";  // idle, running, paused, succeeded, failed, safe_stopped, ...
  std::string task;
  int step = 0;                // 1-based top-level step
  int steps_total = 0;
  std::string step_name;
  std::string item;            // current for_each object
  std::string skill;           // running skill node
  std::string phase;           // its latest feedback phase
  std::string holding;         // object in the gripper ("" = none)
  std::string run_dir;
};

class RunContext
{
public:
  RunContext();

  World world;
  const cell_workspace::Workspace * workspace = nullptr;  // for drop spots and target checks

  // settings, from the executor parameters
  std::map<std::string, std::string> grasp_strategies;    // object class -> Pick strategy
  bool place_compliance = true;  // Place default when not given (bins always use position)

  // counters for the summary
  int steps_succeeded = 0;
  int steps_skipped = 0;
  int items_skipped = 0;
  int skill_calls = 0;
  int skill_failures = 0;
  int failed_step = 0;         // step that ended the task (0 = none)
  bool safe_stopped = false;   // the task failed under on_failure: safe_stop
  std::string last_error;      // most recent skill failure, "Pick: ..."

  // The next drop spot ("bin_a/2") for a bin that has several ("bin_a"), the next slot in row
  // order ("rack_done/A2") for a tray named on its own, or the next slot in one row for a tray
  // and a row letter ("rack_done/B" gives "rack_done/B1", then B2); anything else comes back
  // unchanged.
  // The spots exist because two cubes placed at the bin centre landed on each other. When a bin
  // or tray is full its spots or slots are reused, with a warning.
  std::string allocateTarget(const std::string & target);

  // updates the live progress, thread safe
  void updateProgress(const std::function<void(Progress &)> & update);
  Progress progress() const;

  // seconds since the context was created, which is the run start
  double elapsed() const;

  // Starts writing the run log to `path` as JSON lines. Without it, log() writes nothing.
  void openLog(const std::string & path);
  // appends one record, adding "t", "step" and "item" when they are not set
  void log(Json record);

  // Where messages go, the ROS logger in the executor. The levels are info, warn and error.
  std::function<void(const std::string & level, const std::string & message)> sink;
  void info(const std::string & message) const;
  void warn(const std::string & message) const;
  void error(const std::string & message) const;

private:
  mutable std::mutex mutex_;
  Progress progress_;
  std::map<std::string, int> bin_fill_;
  std::ofstream log_;
  std::chrono::steady_clock::time_point start_;
};

}  // namespace cell_executor
