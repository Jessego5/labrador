// This is the task parser and the tree generator. The parser checks a task file against a small
// schema per step kind and fails with the file's line number, so mistakes in a task file are
// caught before the robot moves. The generator writes the BehaviorTree.CPP v4 XML for a parsed
// task as plain text, which is also saved next to each run as tree.xml.
#include "cell_executor/task.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <set>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace cell_executor
{

namespace
{

// The arguments each step kind accepts. `primary` is the argument a scalar shorthand sets, so
// `- move_to: home` is the same as `- move_to: {pose: home}`.
struct Schema
{
  std::vector<std::string> required;
  std::vector<std::string> optional;
  std::string primary;
};

const std::map<std::string, Schema> & schemas()
{
  static const std::map<std::string, Schema> kSchemas = {
    {"detect", {{}, {"class", "color"}, ""}},
    {"pick", {{"object"}, {"grasp"}, "object"}},
    {"place", {{"target"}, {"compliance"}, "target"}},
    {"insert", {{"target"}, {"max_force", "max_attempts"}, "target"}},
    {"move_to", {{"pose"}, {}, "pose"}},
    {"for_each", {{"do"}, {"filter", "max"}, ""}},
  };
  return kSchemas;
}

const std::set<std::string> kFilterFields = {"class", "color", "type"};
const std::set<std::string> kLoopVariables = {"item", "index"};

int lineOf(const YAML::Node & node)
{
  const int line = node.Mark().line;
  return line >= 0 ? line + 1 : 0;
}

[[noreturn]] void fail(int line, const std::string & message)
{
  throw TaskError(line > 0 ? "line " + std::to_string(line) + ": " + message : message);
}

std::string scalar(const YAML::Node & node, const std::string & what)
{
  if (!node.IsScalar()) {
    fail(lineOf(node), what + ": expected a single value");
  }
  return node.Scalar();
}

std::string join(const std::vector<std::string> & items)
{
  std::string out;
  for (const auto & i : items) {
    out += (out.empty() ? "" : ", ") + i;
  }
  return out;
}

bool isNumber(const std::string & s)
{
  try {
    std::size_t used = 0;
    std::stod(s, &used);
    return used == s.size();
  } catch (const std::exception &) {
    return false;
  }
}

bool isInteger(const std::string & s)
{
  return !s.empty() && s.size() < 10 &&
         s.find_first_not_of("0123456789") == std::string::npos;
}

ObjectFilter parseFilter(const YAML::Node & node, const std::string & what)
{
  ObjectFilter filter;
  if (node.IsNull()) {
    return filter;
  }
  if (!node.IsMap()) {
    fail(lineOf(node), what + ": expected a mapping such as {class: cylinder, color: blue}");
  }
  for (const auto & kv : node) {
    const std::string field = kv.first.as<std::string>();
    if (!kFilterFields.count(field)) {
      fail(lineOf(kv.first), what + ": unknown field '" + field + "' (use class, color or type)");
    }
    filter[field] = scalar(kv.second, what + "." + field);
  }
  return filter;
}

void checkVariables(const std::string & value, bool in_loop, int line, const std::string & what)
{
  for (const auto & v : variablesIn(value)) {
    if (!kLoopVariables.count(v)) {
      fail(line, what + ": unknown variable $" + v + " (only $item and $index exist)");
    }
    if (!in_loop) {
      fail(line, what + ": $" + v + " can only be used inside for_each");
    }
  }
}

Step parseStep(const YAML::Node & node, bool in_loop, const std::string & where);

std::vector<Step> parseSteps(const YAML::Node & node, bool in_loop, const std::string & where)
{
  if (!node.IsSequence() || node.size() == 0) {
    fail(lineOf(node), where + ": expected a non-empty list of steps");
  }
  std::vector<Step> steps;
  for (std::size_t i = 0; i < node.size(); ++i) {
    steps.push_back(parseStep(node[i], in_loop, where + "[" + std::to_string(i) + "]"));
  }
  return steps;
}

Step parseStep(const YAML::Node & node, bool in_loop, const std::string & where)
{
  Step step;
  step.line = lineOf(node);
  YAML::Node value;
  if (node.IsScalar()) {  // "- detect"
    step.kind = node.Scalar();
  } else if (node.IsMap() && node.size() == 1) {
    step.kind = node.begin()->first.as<std::string>();
    value = node.begin()->second;
  } else {
    fail(step.line, where + ": a step is a mapping with one key, e.g. '- pick: {object: $item}'");
  }

  const auto schema_it = schemas().find(step.kind);
  if (schema_it == schemas().end()) {
    std::vector<std::string> kinds;
    for (const auto & s : schemas()) {
      kinds.push_back(s.first);
    }
    fail(step.line, where + ": unknown step '" + step.kind + "' (known: " + join(kinds) + ")");
  }
  const Schema & schema = schema_it->second;
  const std::string what = where + " (" + step.kind + ")";

  if (value && value.IsScalar()) {
    if (schema.primary.empty()) {
      fail(step.line, what + ": expected a mapping of arguments");
    }
    step.args[schema.primary] = value.Scalar();
  } else if (value && value.IsMap()) {
    for (const auto & kv : value) {
      const std::string key = kv.first.as<std::string>();
      const bool known =
        std::count(schema.required.begin(), schema.required.end(), key) ||
        std::count(schema.optional.begin(), schema.optional.end(), key);
      if (!known) {
        std::vector<std::string> all = schema.required;
        all.insert(all.end(), schema.optional.begin(), schema.optional.end());
        fail(lineOf(kv.first), what + ": unknown argument '" + key + "'" +
          (all.empty() ? std::string(" (it takes none)") : " (accepts: " + join(all) + ")"));
      }
      if (key == "filter") {
        step.filter = parseFilter(kv.second, what + ".filter");
      } else if (key == "do") {
        if (in_loop) {
          fail(lineOf(kv.first), what + ": for_each cannot be nested");
        }
        step.body = parseSteps(kv.second, true, where + ".do");
      } else {
        step.args[key] = scalar(kv.second, what + "." + key);
      }
    }
  } else if (value && !value.IsNull()) {
    fail(step.line, what + ": expected a mapping of arguments");
  }

  for (const auto & req : schema.required) {
    if (req == "do" ? step.body.empty() : !step.args.count(req)) {
      fail(step.line, what + ": missing required argument '" + req + "'");
    }
  }
  for (auto & [key, arg] : step.args) {
    if (arg.empty()) {
      fail(step.line, what + "." + key + ": must not be empty");
    }
    checkVariables(arg, in_loop, step.line, what + "." + key);
  }

  // typed arguments
  const auto get = [&](const std::string & key) -> const std::string * {
      const auto it = step.args.find(key);
      return it == step.args.end() ? nullptr : &it->second;
    };
  if (const auto * c = get("compliance"); c && *c != "true" && *c != "false") {
    fail(step.line, what + ".compliance: expected true or false");
  }
  if (const auto * f = get("max_force"); f && (!isNumber(*f) || std::stod(*f) <= 0.0)) {
    fail(step.line, what + ".max_force: expected a positive number (N)");
  }
  if (const auto * a = get("max_attempts");
    a && (!isInteger(*a) || std::stoi(*a) < 1 || std::stoi(*a) > 255))
  {
    fail(step.line, what + ".max_attempts: expected an integer 1..255");
  }
  if (const auto * m = get("max")) {
    if (!isInteger(*m)) {
      fail(step.line, what + ".max: expected a non-negative integer");
    }
    step.max = std::stoi(*m);
    step.args.erase("max");
  }
  if (step.kind == "detect") {
    for (const auto & [key, arg] : step.args) {
      step.filter[key] = arg;  // detect's class and color are a filter for the detector
    }
  }
  return step;
}

std::string xmlEscape(const std::string & text)
{
  std::string out;
  for (const char c : text) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&apos;"; break;
      default: out += c;
    }
  }
  return out;
}

std::string attr(const std::string & name, const std::string & value)
{
  return " " + name + "=\"" + xmlEscape(value) + "\"";
}

std::string argOr(const Step & step, const std::string & key, const std::string & fallback)
{
  const auto it = step.args.find(key);
  return it == step.args.end() ? fallback : it->second;
}

// the skill node for a step that is not a loop
std::string skillXml(const Step & step)
{
  if (step.kind == "detect") {
    return "<DetectObjects" + attr("class", argOr(step, "class", "")) +
           attr("color", argOr(step, "color", "")) + "/>";
  }
  if (step.kind == "pick") {
    return "<Pick" + attr("object", step.args.at("object")) +
           attr("grasp", argOr(step, "grasp", "")) + "/>";
  }
  if (step.kind == "place") {
    return "<Place" + attr("target", step.args.at("target")) +
           attr("compliance", argOr(step, "compliance", "default")) + "/>";
  }
  if (step.kind == "insert") {
    return "<Insert" + attr("target", step.args.at("target")) +
           attr("max_force", argOr(step, "max_force", "0")) +
           attr("max_attempts", argOr(step, "max_attempts", "0")) + "/>";
  }
  if (step.kind == "move_to") {
    return "<MoveToNamedPose" + attr("pose", step.args.at("pose")) + "/>";
  }
  throw std::logic_error("no skill node for step kind '" + step.kind + "'");
}

void stepXml(const Step & step, const std::string & policy, std::ostringstream & out,
  const std::string & indent)
{
  if (step.kind != "for_each") {
    out << indent << skillXml(step) << "\n";
    return;
  }
  out << indent << "<ForEachObject" << attr("filter", encodeFilter(step.filter))
      << attr("max", std::to_string(step.max)) << attr("on_failure", policy) << ">\n";
  out << indent << "  <Sequence name=\"per_object\">\n";
  for (const auto & s : step.body) {
    stepXml(s, policy, out, indent + "    ");
  }
  out << indent << "  </Sequence>\n";
  out << indent << "</ForEachObject>\n";
}

}  // namespace

std::string toString(FailurePolicy policy)
{
  switch (policy) {
    case FailurePolicy::SafeStop: return "safe_stop";
    case FailurePolicy::SkipStep: return "skip_step";
    case FailurePolicy::Abort: return "abort";
  }
  return "safe_stop";
}

FailurePolicy parseFailurePolicy(const std::string & text)
{
  if (text == "safe_stop") {
    return FailurePolicy::SafeStop;
  }
  if (text == "skip_step") {
    return FailurePolicy::SkipStep;
  }
  if (text == "abort") {
    return FailurePolicy::Abort;
  }
  throw std::invalid_argument(
          "unknown failure policy '" + text + "' (use safe_stop, skip_step or abort)");
}

std::string encodeFilter(const ObjectFilter & filter)
{
  std::string out;
  for (const auto & [field, value] : filter) {
    out += (out.empty() ? "" : ";") + field + "=" + value;
  }
  return out;
}

ObjectFilter decodeFilter(const std::string & text)
{
  ObjectFilter filter;
  std::stringstream ss(text);
  std::string part;
  while (std::getline(ss, part, ';')) {
    if (part.empty()) {
      continue;
    }
    const auto eq = part.find('=');
    if (eq == std::string::npos || eq == 0 || eq + 1 == part.size()) {
      throw std::invalid_argument("bad filter term '" + part + "' (expected field=value)");
    }
    const std::string field = part.substr(0, eq);
    if (!kFilterFields.count(field)) {
      throw std::invalid_argument("unknown filter field '" + field + "'");
    }
    filter[field] = part.substr(eq + 1);
  }
  return filter;
}

std::string Step::describe() const
{
  std::string out = kind;
  for (const auto & [key, value] : filter) {
    if (kind == "for_each") {
      out += " " + key + "=" + value;
    }
  }
  if (max > 0) {
    out += " max=" + std::to_string(max);
  }
  for (const auto & [key, value] : args) {
    out += " " + key + "=" + value;
  }
  return out;
}

Task parseTask(const std::string & yaml)
{
  YAML::Node root;
  try {
    root = YAML::Load(yaml);
  } catch (const YAML::Exception & e) {
    throw TaskError(std::string("YAML error: ") + e.what());
  }
  if (!root.IsMap()) {
    throw TaskError("a task file is a mapping with 'name' and 'steps'");
  }
  Task task;
  for (const auto & kv : root) {
    const std::string key = kv.first.as<std::string>();
    if (key != "name" && key != "description" && key != "on_failure" && key != "steps") {
      fail(lineOf(kv.first),
        "unknown key '" + key + "' (accepts: name, description, on_failure, steps)");
    }
  }
  if (!root["name"]) {
    throw TaskError("missing required key 'name'");
  }
  task.name = scalar(root["name"], "name");
  if (task.name.empty()) {
    throw TaskError("name must not be empty");
  }
  if (root["description"]) {
    task.description = scalar(root["description"], "description");
  }
  if (root["on_failure"]) {
    try {
      task.on_failure = parseFailurePolicy(scalar(root["on_failure"], "on_failure"));
    } catch (const std::invalid_argument & e) {
      fail(lineOf(root["on_failure"]), std::string("on_failure: ") + e.what());
    }
  }
  if (!root["steps"]) {
    throw TaskError("missing required key 'steps'");
  }
  task.steps = parseSteps(root["steps"], false, "steps");
  return task;
}

Task loadTaskFile(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    throw TaskError("cannot read task file '" + path + "'");
  }
  std::stringstream ss;
  ss << in.rdbuf();
  try {
    return parseTask(ss.str());
  } catch (const TaskError & e) {
    throw TaskError(path + ": " + e.what());
  }
}

std::string toBehaviorTreeXml(const Task & task)
{
  const std::string policy = toString(task.on_failure);
  const bool safe_stop = task.on_failure == FailurePolicy::SafeStop;
  std::ostringstream out;
  out << "<?xml version=\"1.0\"?>\n";
  out << "<!-- Generated by cell_executor from task '" << xmlEscape(task.name) << "'. -->\n";
  out << "<root BTCPP_format=\"4\" main_tree_to_execute=\"Task\">\n";
  out << "  <BehaviorTree ID=\"Task\">\n";
  std::string indent = "    ";
  if (safe_stop) {
    out << indent << "<Fallback name=\"task_or_safe_stop\">\n";
    indent += "  ";
  }
  out << indent << "<Sequence" << attr("name", task.name) << ">\n";
  for (std::size_t i = 0; i < task.steps.size(); ++i) {
    const Step & step = task.steps[i];
    out << indent << "  <Step" << attr("index", std::to_string(i + 1))
        << attr("title", step.describe()) << attr("on_failure", policy) << ">\n";
    stepXml(step, policy, out, indent + "    ");
    out << indent << "  </Step>\n";
  }
  out << indent << "</Sequence>\n";
  if (safe_stop) {
    out << indent << "<Sequence name=\"safe_stop\">\n";
    out << indent << "  <SubTree ID=\"SafeStop\"/>\n";
    out << indent << "  <AlwaysFailure/>\n";
    out << indent << "</Sequence>\n";
    indent.resize(indent.size() - 2);
    out << indent << "</Fallback>\n";
  }
  out << "  </BehaviorTree>\n";
  out << "</root>\n";
  return out.str();
}

std::vector<std::string> variablesIn(const std::string & text)
{
  std::vector<std::string> names;
  for (std::size_t i = 0; i < text.size(); ++i) {
    if (text[i] != '$' || i + 1 >= text.size()) {
      continue;
    }
    std::size_t j = i + 1;
    if (!(std::isalpha(static_cast<unsigned char>(text[j])) || text[j] == '_')) {
      continue;
    }
    while (j < text.size() && (std::isalnum(static_cast<unsigned char>(text[j])) || text[j] == '_')) {
      ++j;
    }
    names.push_back(text.substr(i + 1, j - i - 1));
    i = j - 1;
  }
  return names;
}

std::string expandVariables(
  const std::string & text,
  const std::function<std::optional<std::string>(const std::string &)> & lookup)
{
  std::string out;
  for (std::size_t i = 0; i < text.size(); ++i) {
    const bool starts_name = text[i] == '$' && i + 1 < text.size() &&
      (std::isalpha(static_cast<unsigned char>(text[i + 1])) || text[i + 1] == '_');
    if (!starts_name) {
      out += text[i];
      continue;
    }
    std::size_t j = i + 1;
    while (j < text.size() && (std::isalnum(static_cast<unsigned char>(text[j])) || text[j] == '_')) {
      ++j;
    }
    const std::string name = text.substr(i + 1, j - i - 1);
    const auto value = lookup(name);
    if (!value) {
      throw std::runtime_error("variable $" + name + " is not set");
    }
    out += *value;
    i = j - 1;
  }
  return out;
}

}  // namespace cell_executor
