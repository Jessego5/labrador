// This is the implementation of the run's shared state: the world model the skill nodes update,
// the drop-spot allocation for bins, and the run log. The progress is behind a mutex because
// /executor/state reads it from a ROS thread while the tree ticks on another.
#include "cell_executor/run_context.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace cell_executor
{

namespace
{

// ("blue_cylinder", 2) for "blue_cylinder_2", or nullopt when there is no numeric suffix
std::optional<std::pair<std::string, int>> splitIndex(const std::string & ref)
{
  const auto underscore = ref.rfind('_');
  if (underscore == std::string::npos || underscore == 0 || underscore + 1 == ref.size()) {
    return std::nullopt;
  }
  const std::string digits = ref.substr(underscore + 1);
  if (digits.size() > 6 || !std::all_of(digits.begin(), digits.end(), [](unsigned char c) {
      return std::isdigit(c);
    }))
  {
    return std::nullopt;
  }
  return std::make_pair(ref.substr(0, underscore), std::stoi(digits));
}

std::vector<const DetectedObject *> sortedById(const std::vector<DetectedObject> & objects)
{
  std::vector<const DetectedObject *> sorted;
  for (const auto & o : objects) {
    sorted.push_back(&o);
  }
  std::sort(sorted.begin(), sorted.end(), [](const auto * a, const auto * b) {
      return a->id < b->id;
    });
  return sorted;
}

}  // namespace

// World

void World::updateDetections(
  const std::vector<DetectedObject> & objects, const ObjectFilter & filter)
{
  objects_.erase(
    std::remove_if(objects_.begin(), objects_.end(), [&](const DetectedObject & o) {
      return matches(o, filter);
    }),
    objects_.end());
  objects_.insert(objects_.end(), objects.begin(), objects.end());
}

bool World::matches(const DetectedObject & object, const ObjectFilter & filter)
{
  for (const auto & [field, value] : filter) {
    const std::string & actual =
      field == "class" ? object.object_class : field == "color" ? object.color : object.type;
    if (actual != value) {
      return false;
    }
  }
  return true;
}

std::optional<DetectedObject> World::find(const std::string & ref) const
{
  for (const auto & o : objects_) {
    if (o.id == ref) {
      return o;
    }
  }
  const auto split = splitIndex(ref);
  if (!split) {
    return std::nullopt;
  }
  const auto & [name, n] = *split;
  for (const char * field : {"class", "type"}) {
    int seen = 0;
    for (const auto * o : sortedById(objects_)) {
      if (matches(*o, {{field, name}}) && seen++ == n) {
        return *o;
      }
    }
  }
  return std::nullopt;
}

std::vector<std::string> World::select(const ObjectFilter & filter, int max) const
{
  std::vector<std::string> ids;
  for (const auto * o : sortedById(objects_)) {
    if (matches(*o, filter) && !handled(o->id) && held != o->id) {
      ids.push_back(o->id);
      if (max > 0 && static_cast<int>(ids.size()) >= max) {
        break;
      }
    }
  }
  return ids;
}

// RunContext

RunContext::RunContext()
: start_(std::chrono::steady_clock::now())
{
}

std::string RunContext::allocateTarget(const std::string & target)
{
  if (!workspace || target.find('/') != std::string::npos) {
    return target;
  }
  const auto * bin = workspace->findBin(target);
  if (!bin || bin->spotCount() <= 1) {
    return target;
  }
  int & used = bin_fill_[target];
  ++used;
  if (used > bin->spotCount()) {
    warn("bin '" + target + "' is full (" + std::to_string(bin->spotCount()) +
      " drop spots); reusing a spot");
  }
  return target + "/" + std::to_string((used - 1) % bin->spotCount() + 1);
}

void RunContext::updateProgress(const std::function<void(Progress &)> & update)
{
  std::lock_guard<std::mutex> lock(mutex_);
  update(progress_);
}

Progress RunContext::progress() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return progress_;
}

double RunContext::elapsed() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
}

void RunContext::openLog(const std::string & path)
{
  log_.open(path, std::ios::out | std::ios::trunc);
  if (!log_) {
    warn("cannot write the run log " + path);
  }
}

void RunContext::log(Json record)
{
  const Progress p = progress();
  if (!record.contains("t")) {
    record["t"] = std::round(elapsed() * 1000.0) / 1000.0;
  }
  if (!record.contains("step")) {
    record["step"] = p.step;
  }
  if (!record.contains("item") && !p.item.empty()) {
    record["item"] = p.item;
  }
  if (log_) {
    log_ << record.dump() << "\n";
    log_.flush();
  }
}

void RunContext::info(const std::string & message) const
{
  if (sink) {
    sink("info", message);
  }
}

void RunContext::warn(const std::string & message) const
{
  if (sink) {
    sink("warn", message);
  }
}

void RunContext::error(const std::string & message) const
{
  if (sink) {
    sink("error", message);
  }
}

}  // namespace cell_executor
