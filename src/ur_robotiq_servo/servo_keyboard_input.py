#!/usr/bin/env python3

import sys
import tty
import termios
import threading
import signal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog

# Define key codes
KEYCODE_RIGHT = 0x43
KEYCODE_LEFT = 0x44
KEYCODE_UP = 0x41
KEYCODE_DOWN = 0x42
KEYCODE_PERIOD = 0x2E
KEYCODE_SEMICOLON = 0x3B
KEYCODE_1 = 0x31
KEYCODE_2 = 0x32
KEYCODE_3 = 0x33
KEYCODE_4 = 0x34
KEYCODE_5 = 0x35
KEYCODE_6 = 0x36
KEYCODE_7 = 0x37
KEYCODE_Q = 0x71
KEYCODE_R = 0x72
KEYCODE_J = 0x6A
KEYCODE_T = 0x74
KEYCODE_W = 0x77
KEYCODE_E = 0x65

# Constants used in the Servo Teleop demo
TWIST_TOPIC = "/servo_node/delta_twist_cmds"
JOINT_TOPIC = "/servo_node/delta_joint_cmds"
ROS_QUEUE_SIZE = 10
PLANNING_FRAME_ID = "world"
EE_FRAME_ID = "tool0"


class KeyboardReader:
    def __init__(self):
        self.file_descriptor = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.file_descriptor)
        tty.setraw(self.file_descriptor)

    def read_one(self):
        return sys.stdin.read(1)

    def shutdown(self):
        termios.tcsetattr(self.file_descriptor, termios.TCSADRAIN, self.old_settings)


class KeyboardServo:
    def __init__(self):
        self.joint_vel_cmd = 1.0
        self.command_frame_id = PLANNING_FRAME_ID
        self.node = rclpy.create_node("servo_keyboard_input")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST_TOPIC, ROS_QUEUE_SIZE)
        self.joint_pub = self.node.create_publisher(JointJog, JOINT_TOPIC, ROS_QUEUE_SIZE)
        self.switch_input = self.node.create_client(ServoCommandType, "servo_node/switch_command_type")
        self.request = ServoCommandType.Request()

    def spin(self):
        rclpy.spin(self.node)

    def key_loop(self):
        publish_twist = False
        publish_joint = False
        print("Reading from keyboard")
        print("---------------------------")
        print("All commands are in the planning frame")
        print("Use arrow keys and the '.' and ';' keys to Cartesian jog")
        print("Use 1|2|3|4|5|6|7 keys to joint jog. 'r' to reverse the direction of jogging.")
        print("Use 'j' to select joint jog.")
        print("Use 't' to select twist")
        print("Use 'w' and 'e' to switch between sending command in planning frame or end effector frame")
        print("'Q' to quit.")

        while True:
            c = input()

            twist_msg = TwistStamped()
            joint_msg = JointJog()
            joint_msg.joint_names = [
                "shoulder_lift_joint",
                "shoulder_pan_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_2_joint",
                "finger_joint",
            ]
            joint_msg.velocities = [0.0] * 7

            # Use read key-press
            if c == '\x1b':  # ANSI escape sequence
                c = input()
                if c == '[':
                    c = input()
                    if c == 'A':
                        twist_msg.twist.linear.x = 0.5  # UP
                        publish_twist = True
                    elif c == 'B':
                        twist_msg.twist.linear.x = -0.5  # DOWN
                        publish_twist = True
                    elif c == 'C':
                        twist_msg.twist.linear.y = 0.5  # RIGHT
                        publish_twist = True
                    elif c == 'D':
                        twist_msg.twist.linear.y = -0.5  # LEFT
                        publish_twist = True
            elif ord(c) == KEYCODE_PERIOD:
                twist_msg.twist.linear.z = -0.5  # '.'
                publish_twist = True
            elif ord(c) == KEYCODE_SEMICOLON:
                twist_msg.twist.linear.z = 0.5  # ';'
                publish_twist = True
            elif ord(c) == KEYCODE_1:
                joint_msg.velocities[0] = self.joint_vel_cmd  # '1'
                publish_joint = True
            elif ord(c) == KEYCODE_2:
                joint_msg.velocities[1] = self.joint_vel_cmd  # '2'
                publish_joint = True
            elif ord(c) == KEYCODE_3:
                joint_msg.velocities[2] = self.joint_vel_cmd  # '3'
                publish_joint = True
            elif ord(c) == KEYCODE_4:
                joint_msg.velocities[3] = self.joint_vel_cmd  # '4'
                publish_joint = True
            elif ord(c) == KEYCODE_5:
                joint_msg.velocities[4] = self.joint_vel_cmd  # '5'
                publish_joint = True
            elif ord(c) == KEYCODE_6:
                joint_msg.velocities[5] = self.joint_vel_cmd  # '6'
                publish_joint = True
            elif ord(c) == KEYCODE_7:
                joint_msg.velocities[6] = self.joint_vel_cmd  # '7'
                publish_joint = True
            elif ord(c) == KEYCODE_R:
                self.joint_vel_cmd *= -1  # 'r'
            elif ord(c) == KEYCODE_J:
                self.request.command_type = ServoCommandType.Request.JOINT_JOG  # 'j'
                if self.switch_input.wait_for_service(timeout_sec=1):
                    result = self.switch_input.call(self.request)
                    if result.success:
                        print("Switched to input type: JointJog")
                    else:
                        print("Could not switch input to: JointJog")
            elif ord(c) == KEYCODE_T:
                self.request.command_type = ServoCommandType.Request.TWIST  # 't'
                if self.switch_input.wait_for_service(timeout_sec=1):
                    result = self.switch_input.call(self.request)
                    if result.success:
                        print("Switched to input type: Twist")
                    else:
                        print("Could not switch input to: Twist")
            elif ord(c) == KEYCODE_W:
                print(f"Command frame set to: {PLANNING_FRAME_ID}")  # 'w'
                self.command_frame_id = PLANNING_FRAME_ID
            elif ord(c) == KEYCODE_E:
                print(f"Command frame set to: {EE_FRAME_ID}")  # 'e'
                self.command_frame_id = EE_FRAME_ID
            elif ord(c) == KEYCODE_Q:
                print("Quit")  # 'Q'
                return 0

            # If a key requiring a publish was pressed, publish the message now
            if publish_twist:
                twist_msg.header.stamp = self.node.get_clock().now().to_msg()
                twist_msg.header.frame_id = self.command_frame_id
                self.twist_pub.publish(twist_msg)
                publish_twist = False
            elif publish_joint:
                joint_msg.header.stamp = self.node.get_clock().now().to_msg()
                joint_msg.header.frame_id = PLANNING_FRAME_ID
                self.joint_pub.publish(joint_msg)
                publish_joint = False

        return 0


def quit_handler(sig, frame):
    input_reader.shutdown()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, quit_handler)
    rclpy.init(args=sys.argv)
    input_reader = KeyboardReader()
    servo_keyboard = KeyboardServo()
    threading.Thread(target=servo_keyboard.spin).start()
    servo_keyboard.key_loop()
