import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout


class PS5ControllerNode(Node):
    def __init__(self):
        super().__init__('ps5_controller_node')
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
            self.joy_callback,
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

        self.get_logger().info('ps5_servo_node started!')

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

    def joy_callback(self, msg):
        # Check for button press to toggle mode
        # Assuming button 2 (e.g., Triangle on PS5) for toggling mode
        current_button_state = msg.buttons[16]
        if current_button_state == 1 and self.last_button_state == 0:
            self.mode = 'joint' if self.mode == 'twist' else 'twist'
            self.get_logger().info(f'Mode switched to: {self.mode}')
        self.last_button_state = current_button_state

        left_trigger = (msg.axes[4] - 1) / - 2.0
        right_trigger = (msg.axes[5] - 1) / - 2.0

        # Process control input based on current mode
        if self.mode == 'twist':
            twist_msg = TwistStamped()
            twist_msg.header.frame_id = 'tool0'
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.twist.linear.x = msg.axes[0] * self.linear_speed_multiplier
            twist_msg.twist.linear.y = -msg.axes[1] * self.linear_speed_multiplier
            twist_msg.twist.linear.z = (left_trigger - right_trigger) * self.linear_speed_multiplier
            twist_msg.twist.angular.x = msg.axes[3]
            twist_msg.twist.angular.y = msg.axes[2]
            twist_msg.twist.angular.z = (msg.buttons[9] - msg.buttons[10]) * 1.0
            self.twist_pub.publish(twist_msg)
        elif self.mode == 'joint':
            joint_msg = JointJog()
            joint_msg.header.frame_id = 'tool0'
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.joint_names = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
            ]
            joint_msg.velocities = [msg.axes[0],
                                    msg.axes[1],
                                    msg.axes[2],
                                    msg.axes[3],
                                    (msg.buttons[11] - msg.buttons[12]) * 1.0,
                                    (msg.buttons[13] - msg.buttons[14]) * 1.0]
            self.joint_pub.publish(joint_msg)

        # Gripper controller
        new_finger_joint_angle = (self.last_finger_joint_angle +
                                  (msg.buttons[1] - msg.buttons[0]) * self.gripper_speed_multiplier)
        if self.gripper_lower_limit > new_finger_joint_angle or self.gripper_upper_limit < new_finger_joint_angle:
            self.get_logger().debug(f"New finger joint angle out of bounds: {new_finger_joint_angle}")
            new_finger_joint_angle = self.last_finger_joint_angle
        gripper_msg = Float64MultiArray()
        layout_msg = MultiArrayLayout()
        layout_msg.dim = [MultiArrayDimension()]
        layout_msg.dim[0].label = "finger_joint"
        layout_msg.dim[0].size = 1
        layout_msg.dim[0].stride = 1
        layout_msg.data_offset = 0
        gripper_msg.layout = layout_msg
        gripper_msg.data = [new_finger_joint_angle]
        self.gripper_cmd_pub.publish(gripper_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PS5ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()