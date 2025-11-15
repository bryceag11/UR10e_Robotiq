from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Get arguments
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gripper_com_port = LaunchConfiguration("gripper_com_port")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")

    # Path to our robot description launch file
    description_launchfile = PathJoinSubstitution([
        FindPackageShare("ur10e_robotiq_cell"),
        "launch",
        "ur10e_robotiq_description.launch.py"
    ])

    # Path to controllers configuration with delayed gripper initialization
    controllers_file = PathJoinSubstitution([
        FindPackageShare("ur10e_robotiq_cell"),
        "config",
        "ur10e_robotiq_controllers.yaml"
    ])

    # Include UR control launch file with our custom description
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
            "use_mock_hardware": use_fake_hardware,
            "launch_rviz": launch_rviz,
            "description_launchfile": description_launchfile,
            "controllers_file": controllers_file,
            "use_tool_communication": use_tool_communication,
            "tool_device_name": tool_device_name,
            "tool_tcp_port": tool_tcp_port,
        }.items()
    )

    # Path to grippy.script
    gripper_script_path = PathJoinSubstitution([
        FindPackageShare("ur10e_robotiq_cell"),
        "resources",
        "grippy.script"
    ])

    # URScript gripper controller (uses port 30002 with URCap functions)
    # Delayed start to allow robot driver to connect first
    urscript_gripper_controller = TimerAction(
        period=5.0,  # Wait 5 seconds for robot to connect
        actions=[
            Node(
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
        ]
    )

    return [
        ur_control_launch,
        urscript_gripper_controller,
    ]


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
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to state"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_com_port",
            default_value="/tmp/ttyUR",
            description="COM port for Robotiq gripper (use /tmp/ttyUR for UR tool communication)"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="true",
            description="Enable UR tool communication for gripper (creates virtual serial port via socat)"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="Device name for tool communication virtual serial port"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="TCP port for UR tool communication"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_description"),
                "config/ur10e/default_kinematics.yaml"
            ]),
            description="Kinematics calibration file for the robot"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
