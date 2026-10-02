"""
This brings up the simulated cell without any planning or skills: MuJoCo running inside
mujoco_ros2_control's ros2_control_node, robot_state_publisher, the arm, gripper and wrist F/T
controllers, the workspace node with its TF frames, and RViz. cell.launch.py includes it and adds
MoveIt, the skills, perception and the executor on top. Only one controller_manager may run at a
time, since a new ros2_control_node will pick up a stale robot_state_publisher's
/robot_description, so stop any earlier bring-up before starting this one. Run it with
ros2 launch cell_bringup sim.launch.py for the MuJoCo viewer and RViz, or with
ros2 launch cell_bringup sim.launch.py headless:=true rviz:=false for neither; scene:=lab
runs the lab table (mjcf/lab_scene.xml and config/lab/workspace.yaml) instead of the default one.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


# The scenes the cell can run: the MuJoCo model and the workspace layout for each. cell.launch.py
# has the same table for the files its own nodes read.
SCENES = {
    "default": {"model": "mjcf/scene.xml", "workspace": "config/workspace.yaml",
                "plugins": "config/mujoco_plugins.yaml"},
    "lab": {"model": "mjcf/lab_scene.xml", "workspace": "config/lab/workspace.yaml",
            "plugins": "config/lab/mujoco_plugins.yaml"},
}


def scene_files(context):
    """The SCENES entry for the scene:= argument, or an error naming the scenes there are."""
    scene = LaunchConfiguration("scene").perform(context)
    if scene not in SCENES:
        raise ValueError(f"unknown scene '{scene}', expected one of {', '.join(SCENES)}")
    return SCENES[scene]


def launch_setup(context):
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    files = scene_files(context)

    description_share = FindPackageShare("cell_description")
    controllers_file = PathJoinSubstitution([description_share, "config", "ros2_controllers.yaml"])
    mujoco_plugins_file = PathJoinSubstitution([description_share, files["plugins"]])

    # the URDF from xacro; headless goes on to the MuJoCo hardware interface in its ros2_control
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                PathJoinSubstitution([description_share, "urdf", "cell.urdf.xacro"]),
                " headless:=", headless,
                " mujoco_model:=", PathJoinSubstitution([description_share, files["model"]]),
            ]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # MuJoCo runs inside this node, and when it exits the whole launch shuts down with it
    ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        emulate_tty=True,
        output="both",
        parameters=[
            {"use_sim_time": True},
            ParameterFile(controllers_file),
            ParameterFile(mujoco_plugins_file),
        ],
        on_exit=Shutdown(),
    )

    def spawner(name, *extra):
        """A controller_manager spawner for one controller from ros2_controllers.yaml."""
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--param-file", controllers_file, *extra],
            output="both",
        )

    joint_state_broadcaster = spawner("joint_state_broadcaster")
    # start the arm, gripper and F/T controllers once joint states are flowing
    arm_and_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[spawner("joint_trajectory_controller"), spawner("gripper_controller"),
                     spawner("ft_broadcaster"),
                     # loaded and configured but inactive, the skills switch it in when needed
                     spawner("compliance_controller", "--inactive")],
        )
    )

    # static TF for every tray slot, bin and fixture, plus the ResolveTarget service
    workspace_node = Node(
        package="cell_workspace",
        executable="workspace_node",
        parameters=[{
            "use_sim_time": True,
            "workspace_file": PathJoinSubstitution([description_share, files["workspace"]]),
        }],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("cell_bringup"), "rviz", "sim.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
        output="log",
    )

    return [robot_state_publisher, ros2_control_node, joint_state_broadcaster, arm_and_gripper,
            workspace_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false", description="Run MuJoCo without its viewer window"),
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
        DeclareLaunchArgument("scene", default_value="default", description="default or lab"),
        OpaqueFunction(function=launch_setup),
    ])
