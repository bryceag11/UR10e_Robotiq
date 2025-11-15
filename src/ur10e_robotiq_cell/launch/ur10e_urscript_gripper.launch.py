from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


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
            "gripper_script_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur10e_robotiq_cell"),
                "resources",
                "grippy.script"
            ]),
            description="Path to grippy.script file"
        )
    )

    robot_ip = LaunchConfiguration("robot_ip")
    gripper_script_path = LaunchConfiguration("gripper_script_path")

    # URScript gripper controller node
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

    return LaunchDescription(declared_arguments + [
        urscript_gripper_controller,
    ])
