from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    # Get arguments
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # Path to our combined URDF
    description_package = "ur10e_robotiq_cell"
    description_file = "ur10e_robotiq.urdf.xacro"

    # Include MoveIt launch file with our custom description
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
            "launch_rviz": launch_rviz,
            "use_fake_hardware": use_fake_hardware,
            "description_package": description_package,
            "description_file": description_file,
        }.items()
    )

    return [moveit_launch]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with MoveIt"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
