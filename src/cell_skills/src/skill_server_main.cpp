// This is the standalone skill server, the executable that cell.launch.py runs. The node is also
// registered as a component, but run on its own it needs a multi-threaded executor, so that the
// action, gripper and joint-state callbacks keep flowing while a skill blocks on MoveIt.
#include <memory>

#include "cell_skills/skill_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cell_skills::SkillServer>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
