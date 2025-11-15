from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Get arguments
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_moveit = LaunchConfiguration("launch_moveit")

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
            "launch_rviz": launch_rviz,
            "description_launchfile": description_launchfile,
            "controllers_file": controllers_file,
            "use_tool_communication": "false",  # We use URScript via port 30002 instead
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
        moveit_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("ur_moveit_config"),
                    "launch",
                    "ur_moveit.launch.py"
                ])
            ]),
            launch_arguments={
                "ur_type": "ur10e",
                "launch_rviz": "false",  # Already launched by ur_control
                "description_package": "ur10e_robotiq_cell",
                "description_file": "ur10e_robotiq.urdf.xacro",
            }.items()
        )
        nodes_to_launch.append(moveit_launch)

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
            default_value="false",
            description="Launch MoveIt motion planning"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
