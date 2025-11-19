from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def launch_setup(context, *args, **kwargs):
    # Get arguments
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_moveit = LaunchConfiguration("launch_moveit")
    headless_mode = LaunchConfiguration("headless_mode")

    # Path to our robot description launch file
    description_launchfile = PathJoinSubstitution([
        FindPackageShare("ur10e_robotiq_cell"),
        "launch",
        "ur10e_robotiq_description.launch.py"
    ])

    # Path to controllers configuration
    controllers_file = PathJoinSubstitution([
        FindPackageShare("ur10e_robotiq_cell"),
        "config",
        "ur10e_robotiq_controllers.yaml"
    ])

    # Path to grippy.script
    gripper_script_path = PathJoinSubstitution([
        FindPackageShare("ur10e_robotiq_cell"),
        "resources",
        "grippy.script"
    ])

    nodes_to_launch = []

    # Determine RViz launch strategy:
    # - If MoveIt is enabled, disable RViz in ur_control (MoveIt will launch it with MotionPlanning plugin)
    # - If MoveIt is disabled, launch RViz from ur_control
    ur_control_rviz = "false" if launch_moveit.perform(context) == "true" else launch_rviz.perform(context)

    # 1. Launch UR robot driver
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ]),
        launch_arguments={
            "ur_type": "ur10e",
            "robot_ip": robot_ip,
            "use_mock_hardware": "false",
            "launch_rviz": ur_control_rviz,  # Conditional RViz launch
            "description_launchfile": description_launchfile,
            "controllers_file": controllers_file,
            "use_tool_communication": "false",  # We use URScript via port 30002 instead
            "activate_joint_controller": "true",  # Ensure trajectory controller is activated
            "headless_mode": headless_mode,  # Auto-start External Control program
        }.items()
    )
    nodes_to_launch.append(ur_control_launch)

    # 2. Launch URScript gripper controller (port 30002 with URCap functions)
    urscript_gripper_controller = Node(
        package='ur10e_robotiq_cell',
        executable='urscript_gripper_controller.py',
        name='urscript_gripper_controller',
        output='screen',
        parameters=[{
            'robot_ip': robot_ip,
            'robot_port': 30002,
            'gripper_script_path': gripper_script_path,
        }]
    )
    nodes_to_launch.append(urscript_gripper_controller)

    # 3. Optionally launch MoveIt if requested
    if launch_moveit.perform(context) == 'true':
        # Launch MoveIt without RViz (we'll launch RViz separately with timer)
        moveit_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("ur10e_robotiq_moveit_config"),
                    "launch",
                    "ur_moveit.launch.py"
                ])
            ]),
            launch_arguments={
                "ur_type": "ur10e",
                "launch_rviz": "false",  # Disable RViz in ur_moveit (we launch it separately)
            }.items()
        )
        nodes_to_launch.append(moveit_launch)

        # 4. Launch RViz with MoveIt config (delayed to ensure robot_description is available)
        if launch_rviz.perform(context) == 'true':
            # Build MoveIt configs for RViz
            moveit_config = (
                MoveItConfigsBuilder(robot_name="ur10e_robotiq", package_name="ur10e_robotiq_moveit_config")
                .robot_description(file_path=get_package_share_directory("ur10e_robotiq_cell") + "/urdf/ur10e_robotiq.urdf.xacro")
                .robot_description_semantic(Path(get_package_share_directory("ur10e_robotiq_moveit_config")) / "srdf" / "ur10e_robotiq.srdf.xacro")
                .to_moveit_configs()
            )

            rviz_config_file = PathJoinSubstitution([
                FindPackageShare("ur10e_robotiq_moveit_config"),
                "config",
                "moveit.rviz"
            ])

            # Delayed RViz launch (1 second to ensure robot_description is published)
            rviz_node = TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_moveit",
                        output="log",
                        arguments=["-d", rviz_config_file],
                        parameters=[
                            moveit_config.robot_description,
                            moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics,
                            moveit_config.planning_pipelines,
                            moveit_config.joint_limits,
                        ],
                    )
                ]
            )
            nodes_to_launch.append(rviz_node)

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="IP address of the UR robot"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz for visualization"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_moveit",
            default_value="true",
            description="Launch MoveIt motion planning"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="true",
            description="Automatically start and stop External Control program on robot"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
