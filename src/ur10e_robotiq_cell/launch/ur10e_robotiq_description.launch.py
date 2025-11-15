from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="Type/series of used UR robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur10e_robotiq_cell",
            description="Description package with robot URDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur10e_robotiq.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="tf_prefix of the joint names, useful for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_gripper_hardware",
            default_value="true",
            description="Start gripper with fake hardware mirroring command to state (URScript controller handles real gripper).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_com_port",
            default_value="/dev/ttyUSB0",
            description="COM port for Robotiq gripper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_description"),
                "config/ur10e/default_kinematics.yaml"
            ]),
            description="Kinematics calibration file for the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="0.0.0.0",
            description="IP address of the robot.",
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_gripper_hardware = LaunchConfiguration("use_fake_gripper_hardware")
    gripper_com_port = LaunchConfiguration("gripper_com_port")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    robot_ip = LaunchConfiguration("robot_ip")

    # External Control file paths - using custom script with tool communication enabled
    script_filename = PathJoinSubstitution([
        FindPackageShare("ur10e_robotiq_cell"),
        "resources",
        "external_control_with_tool_comm.urscript"
    ])
    input_recipe_filename = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),
        "resources",
        "rtde_input_recipe.txt"
    ])
    output_recipe_filename = PathJoinSubstitution([
        FindPackageShare("ur_robot_driver"),
        "resources",
        "rtde_output_recipe.txt"
    ])

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "joint_limit_params:=",
            PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]),
            " ",
            "kinematics_params:=",
            kinematics_params_file,
            " ",
            "physical_params:=",
            PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]),
            " ",
            "visual_params:=",
            PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_gripper_hardware:=",
            use_fake_gripper_hardware,
            " ",
            "gripper_com_port:=",
            gripper_com_port,
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    nodes_to_start = [robot_state_publisher_node]

    return LaunchDescription(declared_arguments + nodes_to_start)
