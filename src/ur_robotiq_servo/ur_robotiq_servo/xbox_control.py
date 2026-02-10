import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout


class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox_servo_node')

        # States
        self.mode = 'twist'
        self.last_button_state = 0
        self.last_finger_joint_angle = 0.0

        # Parameters
        self.linear_speed_multiplier = self.declare_parameter('linear_speed_multiplier', 2.0).get_parameter_value().double_value
        self.angular_speed_multiplier = self.declare_parameter('angular_speed_multiplier', 2.0).get_parameter_value().double_value
        self.gripper_speed_multiplier = self.declare_parameter('gripper_speed_multiplier', 0.02).get_parameter_value().double_value
        self.gripper_lower_limit = self.declare_parameter('gripper_lower_limit', 0.0).get_parameter_value().double_value
        self.gripper_upper_limit = self.declare_parameter('gripper_upper_limit', 0.7).get_parameter_value().double_value
        self.use_fake_hardware = self.declare_parameter('use_fake_hardware', False).get_parameter_value().bool_value

        # Axis mapping
        self.axis_linear_x = self.declare_parameter('axis_linear_x', 1).get_parameter_value().integer_value
        self.axis_linear_y = self.declare_parameter('axis_linear_y', 0).get_parameter_value().integer_value
        self.axis_linear_z_up = self.declare_parameter('axis_linear_z_up', 5).get_parameter_value().integer_value
        self.axis_linear_z_down = self.declare_parameter('axis_linear_z_down', 2).get_parameter_value().integer_value
        self.axis_angular_y = self.declare_parameter('axis_angular_y', 4).get_parameter_value().integer_value
        self.axis_angular_z = self.declare_parameter('axis_angular_z', 3).get_parameter_value().integer_value

        # Mode toggle button
        self.toggle_button_index = self.declare_parameter('toggle_button_index', 7).get_parameter_value().integer_value

        self.get_logger().info(f"Linear speed multiplier: {self.linear_speed_multiplier}")
        self.get_logger().info(f"Angular speed multiplier: {self.angular_speed_multiplier}")
        self.get_logger().info(f"Gripper speed multiplier: {self.gripper_speed_multiplier}")
        self.get_logger().info(f"Use fake hardware: {self.use_fake_hardware}")

        # Subscriptions and publishers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        self.gripper_cmd_pub = self.create_publisher(Float64MultiArray, '/forward_gripper_position_controller/commands', 10)

        # Service to start servoing
        self.servo_client = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/servo_node/start_servo service not available, waiting again...')
        self.call_start_servo()

        self.get_logger().info('xbox_servo_node started!')

    def call_start_servo(self):
        request = Trigger.Request()
        future = self.servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Successfully called start_servo: {future.result().message}')
        else:
            self.get_logger().warn('Failed to call start_servo')

    def joint_state_callback(self, msg):
        try:
            index = msg.name.index('finger_joint')
            self.last_finger_joint_angle = msg.position[index]
        except ValueError:
            self.get_logger().debug('finger_joint not found in joint_states')

    def joy_callback(self, msg):
        # Safe read for toggle button
        current_button_state = msg.buttons[self.toggle_button_index] if len(msg.buttons) > self.toggle_button_index else 0
        if current_button_state == 1 and self.last_button_state == 0:
            self.mode = 'joint' if self.mode == 'twist' else 'twist'
            self.get_logger().info(f'Mode switched to: {self.mode}')
        self.last_button_state = current_button_state

        # Safe axis access
        def safe_axis(i): return msg.axes[i] if i < len(msg.axes) else 0.0
        def safe_button(i): return msg.buttons[i] if i < len(msg.buttons) else 0

        # Trigger z control
        z_up = safe_axis(self.axis_linear_z_up)
        z_down = safe_axis(self.axis_linear_z_down)

        if self.mode == 'twist':
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = 'base_link'
            twist.twist.linear.x = safe_axis(self.axis_linear_x) * self.linear_speed_multiplier
            twist.twist.linear.y = -safe_axis(self.axis_linear_y) * self.linear_speed_multiplier
            twist.twist.linear.z = (z_up - z_down) * self.linear_speed_multiplier
            twist.twist.angular.y = safe_axis(self.axis_angular_y) * self.angular_speed_multiplier
            twist.twist.angular.z = safe_axis(self.axis_angular_z) * self.angular_speed_multiplier
            twist.twist.angular.x = 0.0  # not mapped
            self.twist_pub.publish(twist)

        elif self.mode == 'joint':
            joint = JointJog()
            joint.header.stamp = self.get_clock().now().to_msg()
            joint.header.frame_id = 'base_link'
            joint.joint_names = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
            ]
            joint.velocities = [
                safe_axis(self.axis_linear_x),
                safe_axis(self.axis_linear_y),
                safe_axis(self.axis_linear_z_up) - safe_axis(self.axis_linear_z_down),
                safe_axis(self.axis_angular_z),
                safe_button(11) - safe_button(12),
                safe_button(13) - safe_button(14)
            ]
            self.joint_pub.publish(joint)

        # Gripper control
        delta = safe_button(1) - safe_button(0)
        new_finger_angle = self.last_finger_joint_angle + delta * self.gripper_speed_multiplier
        if self.gripper_lower_limit <= new_finger_angle <= self.gripper_upper_limit:
            self.last_finger_joint_angle = new_finger_angle
        gripper_msg = Float64MultiArray()
        gripper_msg.layout = MultiArrayLayout(dim=[
            MultiArrayDimension(label="finger_joint", size=1, stride=1)
        ], data_offset=0)
        gripper_msg.data = [self.last_finger_joint_angle]
        self.gripper_cmd_pub.publish(gripper_msg)


def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
