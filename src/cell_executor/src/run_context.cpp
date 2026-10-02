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

std::vector<World::Change> World::updateDetections(
  const std::vector<DetectedObject> & objects, const ObjectFilter & filter)
{
  std::vector<Change> changes;
  std::vector<bool> seen(objects_.size(), false);
  std::vector<DetectedObject> added;
  std::set<std::string> taken;
  for (const auto & o : objects_) {
    taken.insert(o.id);
  }
  for (DetectedObject d : objects) {
    // the nearest known object of the same class, not held and not matched yet
    int best = -1;
    double best_dist = match_radius;
    for (std::size_t i = 0; i < objects_.size(); ++i) {
      const auto & k = objects_[i];
      if (seen[i] || k.object_class != d.object_class || held == k.id) {
        continue;
      }
      const auto & a = k.pose.pose.position;
      const auto & b = d.pose.pose.position;
      const double dist = std::hypot(a.x - b.x, a.y - b.y);
      if (dist <= best_dist) {
        best = static_cast<int>(i);
        best_dist = dist;
      }
    }
    if (best >= 0) {
      DetectedObject & k = objects_[best];
      seen[best] = true;
      if (k.color != d.color) {
        changes.push_back({k.id, k.color, d.color});
      }
      d.id = k.id;
      k = d;
    } else {
      d.id = uniqueId(d.id, taken);
      taken.insert(d.id);
      added.push_back(d);
    }
  }
  std::vector<DetectedObject> kept;
  for (std::size_t i = 0; i < objects_.size(); ++i) {
    const auto & k = objects_[i];
    if (seen[i] || !matches(k, filter) || held == k.id || handled(k.id)) {
      kept.push_back(k);
    }
  }
  kept.insert(kept.end(), added.begin(), added.end());
  objects_ = std::move(kept);
  return changes;
}

std::string World::uniqueId(const std::string & id, const std::set<std::string> & taken) const
{
  if (!taken.count(id)) {
    return id;
  }
  const auto split = splitIndex(id);
  const std::string base = split ? split->first : id;
  for (int n = 0;; ++n) {
    const std::string candidate = base + "_" + std::to_string(n);
    if (!taken.count(candidate)) {
      return candidate;
    }
  }
}

bool World::meets(const std::string & id, const ObjectCondition & condition) const
{
  const auto o = find(id);
  if (!o) {
    return false;
  }
  for (const auto & [field, vals] : condition) {
    if (field == "hue" || field == "area") {
      // every comparison must hold, so ">=95" and "<=125" together make a range
      const double value = field == "hue" ? o->hue : 1e6 * o->inner_area;  // degrees, mm^2
      for (const auto & v : vals) {
        const bool two = v.size() > 1 && v[1] == '=';
        const double limit = std::stod(v.substr(two ? 2 : 1));
        const bool ok = v[0] == '<' ? (two ? value <= limit : value < limit) :
          (two ? value >= limit : value > limit);
        if (!ok) {
          return false;
        }
      }
      continue;
    }
    const std::string & actual =
      field == "class" ? o->object_class : field == "color" ? o->color : o->type;
    if (std::find(vals.begin(), vals.end(), actual) == vals.end()) {
      return false;
    }
  }
  return true;
}

void World::moveTo(const std::string & id, const Eigen::Vector3d & center)
{
  for (auto & o : objects_) {
    if (o.id == id) {
      o.pose.pose.position.x = center.x();
      o.pose.pose.position.y = center.y();
      o.pose.pose.position.z = center.z();
    }
  }
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
  if (!workspace) {
    return target;
  }
  // "rack_done/A": the next free slot in that row of the tray
  const auto slash = target.find('/');
  if (slash != std::string::npos) {
    const std::string base = target.substr(0, slash), row = target.substr(slash + 1);
    for (const auto & tray : workspace->trays) {
      if (tray.name != base || row.size() != 1 || row[0] < 'A' || row[0] - 'A' >= tray.rows) {
        continue;
      }
      int & used = bin_fill_[target];
      ++used;
      if (used > tray.cols) {
        warn("row " + target + " is full (" + std::to_string(tray.cols) +
          " slots); reusing a slot");
      }
      return base + "/" + cell_workspace::slotName(row[0] - 'A', (used - 1) % tray.cols);
    }
    return target;
  }
  if (const auto * bin = workspace->findBin(target); bin && bin->spotCount() > 1) {
    int & used = bin_fill_[target];
    ++used;
    if (used > bin->spotCount()) {
      warn("bin '" + target + "' is full (" + std::to_string(bin->spotCount()) +
        " drop spots); reusing a spot");
    }
    return target + "/" + std::to_string((used - 1) % bin->spotCount() + 1);
  }
  for (const auto & tray : workspace->trays) {
    if (tray.name != target || tray.rows * tray.cols <= 1) {
      continue;
    }
    const int slots = tray.rows * tray.cols;
    int & used = bin_fill_[target];
    ++used;
    if (used > slots) {
      warn("tray '" + target + "' is full (" + std::to_string(slots) + " slots); reusing a slot");
    }
    const int n = (used - 1) % slots;
    return target + "/" + cell_workspace::slotName(n / tray.cols, n % tray.cols);
  }
  return target;
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
