#!/usr/bin/env python3
"""
URScript-based Robotiq Gripper Controller for ROS2
Uses port 30002 and URCap functions from grippy.script
Compatible with MoveIt via GripperCommand action interface
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
import socket
import time
from pathlib import Path
import threading


class URScriptGripperController(Node):
    def __init__(self):
        super().__init__('urscript_gripper_controller')

        # Parameters
        self.declare_parameter('robot_ip', '192.168.56.101')
        self.declare_parameter('robot_port', 30002)
        self.declare_parameter('gripper_script_path', '')

        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        script_path = self.get_parameter('gripper_script_path').value

        # Socket connection
        self.socket = None
        self.socket_lock = threading.Lock()
        self.gripper_script_header = None
        self.gripper_initialized = False

        # Current gripper position (0.0 = open, 0.085 = closed)
        self.current_position = 0.0
        self.position_lock = threading.Lock()

        # Joint state publisher for RViz visualization
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz

        # Load gripper functions from grippy.script
        self._load_gripper_functions(script_path)

        # Connect to robot
        self.connect()

        # Action server for gripper commands (MoveIt compatibility)
        self._action_server = ActionServer(
            self,
            GripperCommand,
            'robotiq_gripper_controller/gripper_cmd',
            self.execute_gripper_command
        )

        self.get_logger().info(f'URScript Gripper Controller started on {self.robot_ip}:{self.robot_port}')

    def _load_gripper_functions(self, script_path):
        """Load Robotiq gripper function definitions from grippy.script"""
        try:
            if not script_path:
                # Try default locations
                possible_paths = [
                    Path(__file__).parent.parent / 'resources' / 'grippy.script',
                    Path('/home/hippi1o2/ws_robotiq/grippy.script'),
                ]
                for p in possible_paths:
                    if p.exists():
                        script_path = str(p)
                        break

            if not script_path or not Path(script_path).exists():
                self.get_logger().error(f'grippy.script not found at {script_path}')
                return False

            with open(script_path, 'r', encoding='utf-8') as f:
                full_script = f.read()

            # Extract Robotiq Gripper functions
            start_marker = "# begin: URCap Installation Node"
            gripper_marker = "#   Source: Robotiq_Grippers"
            gripper_type_marker = "#   Type: Gripper"

            lines = full_script.split('\n')
            gripper_start = None
            gripper_end = None

            for i, line in enumerate(lines):
                if gripper_type_marker in line:
                    for j in range(i, max(0, i-10), -1):
                        if start_marker in lines[j] and gripper_marker in lines[j+1]:
                            gripper_start = j
                            break
                if gripper_start and "# end: URCap Installation Node" in line and i > gripper_start + 10:
                    gripper_end = i + 1
                    break

            if gripper_start and gripper_end:
                self.gripper_script_header = '\n'.join(lines[gripper_start:gripper_end])
                self.get_logger().info('Loaded Robotiq gripper URCap functions')
                return True
            else:
                self.get_logger().error('Could not find Robotiq gripper functions in grippy.script')
                return False

        except Exception as e:
            self.get_logger().error(f'Error loading gripper functions: {e}')
            return False

    def connect(self):
        """Establish connection to UR robot port 30002"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((self.robot_ip, self.robot_port))
            self.get_logger().info(f'Connected to robot at {self.robot_ip}:{self.robot_port}')

            # Activate gripper on startup
            if self.gripper_script_header:
                self.get_logger().info('Activating gripper...')
                if self.activate_gripper():
                    self.gripper_initialized = True
                    self.get_logger().info('Gripper activated successfully!')
                else:
                    self.get_logger().warn('Gripper activation failed')

            return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot: {e}')
            return False

    def send_script(self, script: str):
        """Send URScript command to robot"""
        try:
            with self.socket_lock:
                if not script.endswith('\n'):
                    script += '\n'
                self.socket.send(script.encode())
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to send script: {e}')
            return False

    def _send_gripper_script(self, commands: str):
        """Send gripper commands with URCap function definitions"""
        if not self.gripper_script_header:
            self.get_logger().error('Gripper functions not loaded')
            return False

        full_script = f"""def gripper_program():
{self.gripper_script_header}

{commands}
end
gripper_program()
"""
        return self.send_script(full_script)

    def activate_gripper(self):
        """Activate Robotiq gripper using URCap functions"""
        commands = """    # Reset and activate all grippers
    rq_activate_all_grippers(True)
"""
        result = self._send_gripper_script(commands)
        time.sleep(3)  # Wait for activation
        return result

    def open_gripper(self, speed: int = 100):
        """Open gripper fully"""
        commands = f"""    # Set speed
    rq_set_speed_norm({speed}, "1")

    # Open and wait
    rq_open_and_wait("1")
"""
        return self._send_gripper_script(commands)

    def close_gripper(self, force: int = 100, speed: int = 100):
        """Close gripper fully"""
        commands = f"""    # Set force and speed
    rq_set_force_norm({force}, "1")
    rq_set_speed_norm({speed}, "1")

    # Close and wait
    rq_close_and_wait("1")
"""
        return self._send_gripper_script(commands)

    def move_gripper(self, position_norm: float, force: int = 100, speed: int = 100):
        """
        Move gripper to specific position
        position_norm: 0.0 (fully open) to 1.0 (fully closed)
        """
        # Convert to 0-100 scale for URCap functions
        position_pct = int(position_norm * 100)
        position_pct = max(0, min(100, position_pct))

        commands = f"""    # Set parameters
    rq_set_force_norm({force}, "1")
    rq_set_speed_norm({speed}, "1")

    # Move to position and wait
    rq_move_and_wait_norm({position_pct}, "1")
"""
        return self._send_gripper_script(commands)

    def execute_gripper_command(self, goal_handle):
        """Execute GripperCommand action (MoveIt compatibility)"""
        self.get_logger().info('Executing gripper command...')

        request = goal_handle.request

        # GripperCommand: position in meters (0.0 = fully open, 0.085 = fully closed for 2F-85)
        # Convert to normalized 0-1 range
        max_opening = 0.085  # 85mm for Robotiq 2F-85
        position_norm = request.command.position / max_opening
        position_norm = max(0.0, min(1.0, position_norm))

        # Convert max_effort (N) to force percentage (approximate)
        force_pct = int((request.command.max_effort / 235.0) * 100)  # 235N max force
        force_pct = max(0, min(100, force_pct))

        self.get_logger().info(f'Moving gripper to position {position_norm:.2f} (force {force_pct}%)')

        # Execute movement
        success = self.move_gripper(position_norm, force=force_pct, speed=100)

        # Update current position for joint state publishing
        if success:
            with self.position_lock:
                self.current_position = request.command.position

        # Provide feedback
        feedback_msg = GripperCommand.Feedback()
        feedback_msg.position = request.command.position
        feedback_msg.effort = request.command.max_effort
        feedback_msg.stalled = False
        feedback_msg.reached_goal = success
        goal_handle.publish_feedback(feedback_msg)

        # Complete action
        if success:
            goal_handle.succeed()
            result = GripperCommand.Result()
            result.position = request.command.position
            result.effort = request.command.max_effort
            result.stalled = False
            result.reached_goal = True
            return result
        else:
            goal_handle.abort()
            result = GripperCommand.Result()
            result.reached_goal = False
            return result

    def publish_joint_states(self):
        """Publish gripper joint states for RViz visualization"""
        with self.position_lock:
            # Robotiq 2F-85: position in meters, convert to joint angle
            # 0.0m (open) = 0.0 rad, 0.085m (closed) = 0.804 rad (max angle)
            joint_angle = (self.current_position / 0.085) * 0.804

            # Clamp to joint limits to avoid MoveIt bounds errors
            joint_angle = max(0.0, min(0.804, joint_angle))

            # Publish all gripper joints including mimic joints
            # Based on robotiq_2f_85_macro.urdf.xacro mimic relationships
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [
                'robotiq_85_left_knuckle_joint',      # Main joint
                'robotiq_85_right_knuckle_joint',     # Mimic: -1 * left_knuckle
                'robotiq_85_left_inner_knuckle_joint',  # Mimic: 1 * left_knuckle (default)
                'robotiq_85_right_inner_knuckle_joint', # Mimic: -1 * left_knuckle
                'robotiq_85_left_finger_tip_joint',   # Mimic: -1 * left_knuckle
                'robotiq_85_right_finger_tip_joint',  # Mimic: 1 * left_knuckle (default)
            ]
            msg.position = [
                joint_angle,        # left_knuckle
                -joint_angle,       # right_knuckle (mimic -1)
                joint_angle,        # left_inner_knuckle (mimic 1)
                -joint_angle,       # right_inner_knuckle (mimic -1)
                -joint_angle,       # left_finger_tip (mimic -1)
                joint_angle,        # right_finger_tip (mimic 1)
            ]
            msg.velocity = []
            msg.effort = []

            self.joint_state_pub.publish(msg)

    def destroy_node(self):
        """Cleanup"""
        if self.socket:
            self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = URScriptGripperController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
