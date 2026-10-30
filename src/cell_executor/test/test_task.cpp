// These are the tests for the task file format: the parser on the real sort_and_insert.yaml and on
// its shorthands, the validation messages and their line numbers, the generated XML with its
// SafeStop fallback, the retries that wrap skills in Recover nodes, variable expansion and the
// filter encoding. They need no ROS and no robot. Run them with colcon test --packages-select
// cell_executor.
#include <string>

#include <gtest/gtest.h>

#include "cell_executor/task.hpp"

using cell_executor::decodeFilter;
using cell_executor::encodeFilter;
using cell_executor::expandVariables;
using cell_executor::FailurePolicy;
using cell_executor::loadTaskFile;
using cell_executor::parseTask;
using cell_executor::Task;
using cell_executor::TaskError;
using cell_executor::toBehaviorTreeXml;
using cell_executor::variablesIn;

namespace
{

// the error message parseTask throws for `yaml`, or "" when it parses
std::string errorFor(const std::string & yaml)
{
  try {
    parseTask(yaml);
  } catch (const TaskError & e) {
    return e.what();
  }
  return "";
}

#define EXPECT_ERROR(yaml, needle) \
  do { \
    const std::string message = errorFor(yaml); \
    EXPECT_NE(message.find(needle), std::string::npos) << "got: '" << message << "'"; \
  } while (0)

}  // namespace

TEST(TaskParse, ReferenceTaskFile)
{
  const Task task = loadTaskFile(std::string(TASKS_DIR) + "/sort_and_insert.yaml");
  EXPECT_EQ(task.name, "sort_and_insert");
  EXPECT_EQ(task.on_failure, FailurePolicy::SafeStop);
  ASSERT_EQ(task.steps.size(), 6u);
  EXPECT_EQ(task.steps[0].kind, "detect");
  EXPECT_EQ(task.steps[1].kind, "for_each");
  EXPECT_EQ(task.steps[1].filter.at("color"), "red");
  EXPECT_EQ(task.steps[1].max, 0);
  ASSERT_EQ(task.steps[1].body.size(), 2u);
  EXPECT_EQ(task.steps[1].body[0].args.at("object"), "$item");
  EXPECT_EQ(task.steps[2].max, 4);
  EXPECT_EQ(task.steps[2].body[1].args.at("target"), "tray_1/C$index");
  EXPECT_EQ(task.steps[3].args.at("object"), "peg_0");
  EXPECT_EQ(task.steps[4].args.at("max_attempts"), "3");
  EXPECT_EQ(task.steps[5].args.at("pose"), "home");
  EXPECT_GT(task.steps[3].line, task.steps[2].line);
}

TEST(TaskParse, ShorthandsAndDefaults)
{
  const Task task = parseTask(R"(
name: t
steps:
  - detect
  - detect: {class: cube, color: red}
  - pick: cube_0
  - place: bin_a
  - move_to: home
)");
  EXPECT_EQ(task.on_failure, FailurePolicy::SafeStop);
  ASSERT_EQ(task.steps.size(), 5u);
  EXPECT_TRUE(task.steps[0].args.empty());
  EXPECT_EQ(task.steps[1].filter.at("class"), "cube");
  EXPECT_EQ(task.steps[2].args.at("object"), "cube_0");
  EXPECT_EQ(task.steps[3].args.at("target"), "bin_a");
  EXPECT_EQ(task.steps[4].args.at("pose"), "home");
  EXPECT_EQ(task.steps[3].describe(), "place target=bin_a");
}

TEST(TaskParse, ReportsErrorsWithLineNumbers)
{
  EXPECT_ERROR("name: t\nsteps:\n  - jump: {}\n", "line 3: steps[0]: unknown step 'jump'");
  EXPECT_ERROR("name: t\nsteps:\n  - pick: {}\n", "missing required argument 'object'");
  EXPECT_ERROR("name: t\nsteps:\n  - pick: {object: a, speed: 2}\n",
    "line 3: steps[0] (pick): unknown argument 'speed' (accepts: object, grasp, retries)");
  EXPECT_ERROR("name: t\non_failure: retry\nsteps: [detect]\n", "line 2: on_failure: unknown");
  EXPECT_ERROR("name: t\nsteps:\n  - pick: {object: $item}\n",
    "$item can only be used inside for_each");
  EXPECT_ERROR(
    "name: t\nsteps:\n  - for_each: {do: [{pick: $thing}]}\n", "unknown variable $thing");
  EXPECT_ERROR(
    "name: t\nsteps:\n  - for_each:\n      do:\n        - for_each: {do: [detect]}\n",
    "for_each and watch cannot be nested");
  EXPECT_ERROR("name: t\nsteps:\n  - for_each: {filter: {shape: box}, do: [detect]}\n",
    "unknown field 'shape'");
  EXPECT_ERROR("name: t\nsteps:\n  - for_each: {filter: {class: cube}}\n",
    "missing required argument 'do'");
  EXPECT_ERROR("name: t\nsteps:\n  - place: {target: bin_a, compliance: maybe}\n",
    "compliance: expected true or false");
  EXPECT_ERROR("name: t\nsteps:\n  - insert: {target: f, max_attempts: 0}\n",
    "max_attempts: expected an integer 1..255");
  EXPECT_ERROR("name: t\nsteps:\n  - insert: {target: f, max_force: -3}\n",
    "max_force: expected a positive number");
  EXPECT_ERROR("name: t\nsteps:\n  - {pick: a, place: b}\n", "a step is a mapping with one key");
  EXPECT_ERROR("name: t\nsteps: []\n", "expected a non-empty list of steps");
  EXPECT_ERROR("steps: [detect]\n", "missing required key 'name'");
  EXPECT_ERROR("name: t\nsteps: [detect]\nfoo: 1\n", "line 3: unknown key 'foo'");
  EXPECT_ERROR("name: [unclosed\n", "YAML error");
  EXPECT_ERROR("name: t\nsteps:\n  - detect: {object: x}\n", "unknown argument 'object'");
  EXPECT_THROW(loadTaskFile("/nonexistent/task.yaml"), TaskError);
}

TEST(TaskXml, WrapsStepsAndAddsTheSafeStopFallback)
{
  const Task task = loadTaskFile(std::string(TASKS_DIR) + "/sort_and_insert.yaml");
  const std::string xml = toBehaviorTreeXml(task);
  EXPECT_NE(xml.find("main_tree_to_execute=\"Task\""), std::string::npos);
  EXPECT_NE(xml.find("<Step index=\"1\" title=\"detect\" on_failure=\"safe_stop\">"),
    std::string::npos) << xml;
  EXPECT_NE(xml.find("<ForEachObject filter=\"class=cylinder\" max=\"4\""), std::string::npos);
  EXPECT_NE(xml.find("<Place target=\"tray_1/C$index\" compliance=\"default\"/>"),
    std::string::npos);
  EXPECT_NE(xml.find("<Insert target=\"peg_fixture\" max_force=\"0\" max_attempts=\"3\"/>"),
    std::string::npos);
  EXPECT_NE(xml.find("<SubTree ID=\"SafeStop\"/>"), std::string::npos);

  Task skip = task;
  skip.on_failure = FailurePolicy::SkipStep;
  const std::string skip_xml = toBehaviorTreeXml(skip);
  EXPECT_EQ(skip_xml.find("SafeStop"), std::string::npos);
  EXPECT_NE(skip_xml.find("on_failure=\"skip_step\""), std::string::npos);
}

TEST(TaskXml, EscapesAttributeValues)
{
  const Task task = parseTask("name: a&b\nsteps:\n  - move_to: {pose: \"x<y\"}\n");
  const std::string xml = toBehaviorTreeXml(task);
  EXPECT_NE(xml.find("pose=\"x&lt;y\""), std::string::npos) << xml;
  EXPECT_NE(xml.find("name=\"a&amp;b\""), std::string::npos);
}

TEST(Variables, FindAndExpand)
{
  EXPECT_EQ(variablesIn("tray_1/C$index"), std::vector<std::string>{"index"});
  EXPECT_EQ(variablesIn("$item-$index $ $1"), (std::vector<std::string>{"item", "index"}));
  const auto lookup = [](const std::string & name) -> std::optional<std::string> {
      if (name == "index") {
        return "3";
      }
      if (name == "item") {
        return "blue_cylinder_1";
      }
      return std::nullopt;
    };
  EXPECT_EQ(expandVariables("tray_1/C$index", lookup), "tray_1/C3");
  EXPECT_EQ(expandVariables("$item", lookup), "blue_cylinder_1");
  EXPECT_EQ(expandVariables("cost: $5", lookup), "cost: $5");
  EXPECT_EQ(expandVariables("no variables", lookup), "no variables");
  EXPECT_THROW(expandVariables("$other", lookup), std::runtime_error);
}

TEST(Filters, RoundTrip)
{
  const cell_executor::ObjectFilter filter = {{"color", "blue"}, {"class", "cylinder"}};
  EXPECT_EQ(encodeFilter(filter), "class=cylinder;color=blue");
  EXPECT_EQ(decodeFilter(encodeFilter(filter)), filter);
  EXPECT_TRUE(decodeFilter("").empty());
  EXPECT_THROW(decodeFilter("class"), std::invalid_argument);
  EXPECT_THROW(decodeFilter("size=4"), std::invalid_argument);
}

TEST(TaskParse, LabSteps)
{
  const Task task = parseTask(R"(
name: lab
steps:
  - detect
  - watch:
      filter: {class: vial}
      when: {color: [green, brown]}
      period: 2
      do:
        - pick: $item
        - if: {color: green, then: [{place: rack_done}], else: [{place: waste}]}
  - for_each:
      filter: {color: yellow}
      do:
        - repeat_until: {color: red, do: [dispense, {wait: 1.5}, detect]}
)");
  ASSERT_EQ(task.steps.size(), 3u);
  const auto & watch = task.steps[1];
  EXPECT_EQ(watch.condition.at("color"), (std::vector<std::string>{"green", "brown"}));
  EXPECT_EQ(watch.filter.at("class"), "vial");
  const auto & branch = watch.body[1];
  EXPECT_EQ(branch.kind, "if");
  EXPECT_EQ(branch.args.at("item"), "$item");
  EXPECT_EQ(branch.body[0].args.at("target"), "rack_done");
  EXPECT_EQ(branch.else_body[0].args.at("target"), "waste");
  const auto & repeat = task.steps[2].body[0];
  EXPECT_EQ(repeat.max, 10);  // the default
  EXPECT_EQ(repeat.body[1].args.at("seconds"), "1.5");

  const std::string xml = toBehaviorTreeXml(task);
  EXPECT_NE(xml.find("<WatchObjects filter=\"class=vial\" when=\"color=brown,green\" "
    "period=\"2\""), std::string::npos) << xml;
  EXPECT_NE(xml.find("<ObjectIs item=\"$item\" condition=\"color=green\"/>"),
    std::string::npos);
  EXPECT_NE(xml.find("<RepeatUntilObject item=\"$item\" condition=\"color=red\" max=\"10\" "
    "on_max=\"fail\">"), std::string::npos);
  EXPECT_NE(xml.find("<Sleep msec=\"1500\"/>"), std::string::npos);
  EXPECT_NE(xml.find("<Dispense/>"), std::string::npos);
}

TEST(TaskParse, LabStepErrors)
{
  EXPECT_ERROR("name: t\nsteps:\n  - if: {color: green, then: [detect]}\n",
    "needs an object to test");
  EXPECT_ERROR(
    "name: t\nsteps:\n  - watch: {when: {color: green}, do: [{for_each: {do: [detect]}}]}\n",
    "for_each and watch cannot be nested");
  EXPECT_ERROR("name: t\nsteps:\n  - watch: {do: [detect]}\n",
    "missing required argument 'when'");
  EXPECT_ERROR("name: t\nsteps:\n  - wait: -1\n", "expected a positive number of seconds");
  EXPECT_ERROR("name: t\nsteps:\n  - repeat_until: {item: v, color: red, max: 0, do: [detect]}\n",
    "max: expected a positive integer");
  EXPECT_ERROR("name: t\nsteps:\n  - watch: {when: {shade: dark}, do: [detect]}\n",
    "unknown field 'shade'");
  EXPECT_ERROR(
    "name: t\nsteps:\n  - repeat_until: {item: v, color: red, on_max: retry, do: [detect]}\n",
    "on_max: expected fail or continue");
}

TEST(TaskParse, NumericConditions)
{
  const Task task = parseTask(R"(
name: t
steps:
  - detect: {frames: 4}
  - for_each:
      filter: {class: tube}
      do:
        - if: {hue: ["<=125", ">=95"], then: [detect]}
)");
  EXPECT_EQ(task.steps[0].args.at("frames"), "4");
  EXPECT_EQ(task.steps[1].body[0].condition.at("hue").size(), 2u);
  EXPECT_NE(toBehaviorTreeXml(task).find("frames=\"4\""), std::string::npos);
  EXPECT_ERROR("name: t\nsteps:\n  - for_each: {do: [{if: {hue: blue, then: [detect]}}]}\n",
    "expected a comparison");
  EXPECT_ERROR("name: t\nsteps:\n  - detect: {frames: 0}\n", "frames: expected an integer 1..30");
  EXPECT_ERROR("name: t\nsteps:\n  - for_each: {do: [{if: {then: [detect]}}]}\n",
    "needs something to test");
}

TEST(Conditions, RoundTrip)
{
  const cell_executor::ObjectCondition c = {{"color", {"green", "brown"}}};
  EXPECT_EQ(cell_executor::encodeCondition(c), "color=brown,green");
  EXPECT_EQ(cell_executor::decodeCondition("color=brown,green").at("color").size(), 2u);
  EXPECT_THROW(cell_executor::decodeCondition("color="), std::invalid_argument);
}

TEST(TaskParse, RetriesWrapSkillsInARecoverNode)
{
  // the task's retries reach every pick, place and insert, and a step can set its own
  const Task task = parseTask(
    "name: t\nretries: 2\nsteps:\n  - detect\n  - pick: red_cube_0\n"
    "  - place: {target: bin_a, retries: 0}\n");
  EXPECT_EQ(task.retries, 2);
  EXPECT_EQ(task.steps[1].args.at("retries"), "2");
  EXPECT_EQ(task.steps[2].args.at("retries"), "0");
  EXPECT_EQ(task.steps[0].args.count("retries"), 0u);
  const std::string xml = toBehaviorTreeXml(task);
  EXPECT_NE(xml.find("<Recover attempts=\"3\" what=\"pick\">"), std::string::npos);
  EXPECT_EQ(xml.find("what=\"place\""), std::string::npos);  // retries: 0, a plain Place
  EXPECT_THROW(parseTask("name: t\nretries: 11\nsteps: [detect]\n"), TaskError);
  EXPECT_THROW(parseTask("name: t\nsteps:\n  - pick: {object: a, retries: x}\n"), TaskError);
}
