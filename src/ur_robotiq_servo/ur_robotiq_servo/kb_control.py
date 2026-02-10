import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
import keyboard

class KBControllerNode(Node):
    def __init__(self):
        super().__init__('kb_controller_node')
        # States
        self.mode = 'twist'  # Initialize mode to 'twist'. Alternatives: 'twist', 'joint'
        self.last_button_state = 0  # Track the last state of the toggle button to detect presses
        self.last_finger_joint_angle = 0.0

        # Parameters
        self.linear_speed_multiplier = self.declare_parameter('linear_speed_multiplier', 1.0)
        self.linear_speed_multiplier = self.get_parameter('linear_speed_multiplier').get_parameter_value().double_value
        self.get_logger().info(f"Linear speed multiplier: {self.linear_speed_multiplier}")

        self.use_fake_hardware = self.declare_parameter('use_fake_hardware', False)
        self.use_fake_hardware = self.get_parameter('use_fake_hardware').get_parameter_value().bool_value
        self.get_logger().info(f"Use fake hardware: {self.use_fake_hardware}")

        self.gripper_speed_multiplier = self.declare_parameter('gripper_speed_multiplier', 1.0)
        self.gripper_speed_multiplier = (self.get_parameter('gripper_speed_multiplier')
                                         .get_parameter_value().double_value)
        self.get_logger().info(f"Gripper speed multiplier: {self.gripper_speed_multiplier}")

        self.gripper_lower_limit = self.declare_parameter('gripper_lower_limit', 1.0)
        self.gripper_lower_limit = (self.get_parameter('gripper_lower_limit')
                                         .get_parameter_value().double_value)
        self.get_logger().info(f"Gripper lower limit: {self.gripper_lower_limit}")

        self.gripper_upper_limit = self.declare_parameter('gripper_upper_limit', 1.0)
        self.gripper_upper_limit = (self.get_parameter('gripper_upper_limit')
                                         .get_parameter_value().double_value)
        self.get_logger().info(f"Gripper upper limit: {self.gripper_upper_limit}")
        # Subscriber and Publisher
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.keyboard_callback,
            10)

        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10)

        self.joint_pub = self.create_publisher(
            JointJog,
            '/servo_node/delta_joint_cmds',
            10)

        self.gripper_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_gripper_position_controller/commands',
            10)

        # Services
        self.servo_client = self.create_client(Trigger, '/servo_node/start_servo')

        srv_msg = Trigger.Request()
        while not self.servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/servo_node/start_servo service not available, waiting again...')

        self.call_start_servo()

        self.get_logger().info('kb_servo_node started!')

    def call_start_servo(self):
        request = Trigger.Request()
        future = self.servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info(f'Successfully called start_servo: {response.message}')
        else:
            self.get_logger().info('Failed to call start_servo')

    def joint_state_callback(self, msg):
        try:
            index = msg.name.index('finger_joint')
            self.last_finger_joint_angle = msg.position[index]
        except ValueError:
            self.get_logger().error('finger_joint not found in /joint_states msg')

    def keyboard_callback(self, event):
        # Process keyboard events
        print("Key pressed")
        if event.event_type == keyboard.KEY_DOWN:
            # Handle key presses
            if event.name == 't':
                # Toggle mode between twist and joint control
                self.mode = 'joint' if self.mode == 'twist' else 'twist'
                self.get_logger().info(f'Mode switched to: {self.mode}')
            elif event.name == 'w':
                # Move forward
                self.publish_twist(1.0 * self.linear_speed_multiplier)  # Adjust speed as needed
                self.get_logger().info('Moving forward')
            elif event.name == 's':
                # Move backward
                self.publish_twist(-1.0 * self.linear_speed_multiplier)  # Adjust speed as needed
                self.get_logger().info('Moving backward')
        elif event.event_type == keyboard.KEY_UP:
            # Handle key releases
            if event.name == 'w' or event.name == 's':
                # Stop moving
                self.publish_twist(0.0)
                self.get_logger().info('Stopped moving')

    def publish_twist(self, linear_speed):
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = 'tool0'
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = linear_speed
        twist_msg.twist.linear.y = 0.0  # Adjust as needed
        twist_msg.twist.linear.z = 0.0  # Adjust as needed
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.twist_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KBControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()