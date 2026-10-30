"""
This brings up the whole cell: the simulation from sim.launch.py, MoveIt's move_group, the skill
server, the perception node, the task executor and, on the default table, the fault injector from
cell_monitor. It is what the benchmarks and the task files run against. Run it with ros2 launch
cell_bringup cell.launch.py, add headless:=true rviz:=false to run without the MuJoCo viewer and
RViz, or add task:=sort_and_insert to have the executor run that task as soon as it starts.
scene:=lab runs the lab table instead: every node then reads the lab's layout and object types, and
cell_lab starts to run the samples' chemistry.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_configs():
    """
    The MoveIt configuration from cell_moveit_config, on the same URDF the simulation uses.
    OMPL is the default planner and Pilz is loaded beside it.
    """
    description = get_package_share_directory("cell_description")
    return (
        MoveItConfigsBuilder("cell", package_name="cell_moveit_config")
        .robot_description(file_path=os.path.join(description, "urdf", "cell.urdf.xacro"))
        .robot_description_semantic(file_path="config/cell.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"], default_planning_pipeline="ompl")
        # robot_state_publisher in sim.launch.py already publishes the URDF
        .planning_scene_monitor(
            publish_robot_description=False, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )


# The files each scene's nodes read: the workspace layout (skills, executor), the object types
# and colour thresholds (perception), and whether the lab's chemistry node runs. sim.launch.py
# has the matching table for the MuJoCo model.
SCENES = {
    "default": {"workspace": "config/workspace.yaml", "objects": "config/objects.yaml",
                "perception": "perception.yaml", "lab": False, "color_noise": 0.0},
    # the lab adds camera noise, which the test readout's second look exists to average out
    "lab": {"workspace": "config/lab/workspace.yaml", "objects": "config/lab/lab.yaml",
            "perception": "perception_lab.yaml", "lab": True, "color_noise": 0.05},
}


def launch_setup(context):
    scene = LaunchConfiguration("scene").perform(context)
    if scene not in SCENES:
        raise ValueError(f"unknown scene '{scene}', expected one of {', '.join(SCENES)}")
    files = SCENES[scene]
    description = get_package_share_directory("cell_description")
    workspace_file = os.path.join(description, files["workspace"])
    moveit = moveit_configs()
    sim_time = {"use_sim_time": True}

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("cell_bringup"), "launch", "sim.launch.py")),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "rviz": LaunchConfiguration("rviz"),
            "scene": scene,
        }.items(),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit.to_dict(), sim_time],
    )

    # the skill server has its own MoveGroupInterface, so it needs the same robot model
    skills = Node(
        package="cell_skills",
        executable="skill_server",
        output="screen",
        parameters=[
            moveit.robot_description,
            moveit.robot_description_semantic,
            moveit.robot_description_kinematics,
            moveit.joint_limits,
            sim_time,
            {"workspace_file": workspace_file},
        ],
    )

    perception = Node(
        package="cell_perception",
        executable="perception_node",
        output="screen",
        parameters=[sim_time, {
            "objects_file": os.path.join(description, files["objects"]),
            "perception_file": os.path.join(
                get_package_share_directory("cell_perception"), "config", files["perception"]),
            "color_noise": files["color_noise"],
        }],
    )

    executor = Node(
        package="cell_executor",
        executable="executor_node",
        output="screen",
        parameters=[{"task": LaunchConfiguration("task"), "workspace_file": workspace_file}],
    )

    nodes = [sim, move_group, skills, perception, executor]
    if not files["lab"]:
        # the fault injector, idle until a reliability benchmark starts a run; its layout rules
        # are the default table's, so the lab scene goes without it
        nodes.append(Node(package="cell_monitor", executable="monitor_node", output="screen",
                          parameters=[sim_time]))
    if files["lab"]:
        # the lab's chemistry, which sets the samples out and changes them as they react
        nodes.append(Node(
            package="cell_lab",
            executable="lab_node",
            output="screen",
            parameters=[sim_time, {
                "lab_file": os.path.join(description, files["objects"]),
                "workspace_file": workspace_file,
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("scene", default_value="default", description="default or lab"),
        DeclareLaunchArgument(
            "task", default_value="",
            description="task file (name in cell_executor/tasks or a path) to run at startup"),
        OpaqueFunction(function=launch_setup),
    ])
