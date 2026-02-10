/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : servo_keyboard_input.cpp
 *      Project   : moveit2_tutorials
 *      Created   : 05/31/2021
 *      Author    : Adam Pettinger
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp> // Add necessary includes
#include <std_msgs/msg/float64_multi_array.hpp> // Add necessary includes
#include <sensor_msgs/msg/joint_state.hpp> // Add necessary include for JointState message
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Define used keys
#define KEYCODE_LEFT 0x44
#define KEYCODE_RIGHT 0x43
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_E 0x65
#define KEYCODE_W 0x77
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_R 0x72
#define KEYCODE_Q 0x71
#define KEYCODE_PLUS 0x2B   // Keycode for the plus sign (+)
#define KEYCODE_MINUS 0x2D  // Keycode for the minus sign (-)
#define KEYCODE_GRIPPER 0x67 // Keycode for the gripper control button (g)

// Some constants used in the Servo Teleop demo
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string GRIPPER_TOPIC = "/forward_gripper_position_controller/commands";
const size_t ROS_QUEUE_SIZE = 10;
const std::string EEF_FRAME_ID = "world";
const std::string BASE_FRAME_ID = "tool0";

// A class for reading the key inputs from the terminal
class KeyboardReader
{
public:
  KeyboardReader() : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }
  void readOne(char* c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }
  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class KeyboardServo
{
public:
  KeyboardServo();
  int keyLoop();

private:
  void spin();

  void handlePlusPress();
  void handleMinusPress();
  void jointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg); // Declaration of jointStateCallback
  void publishGripperCommand(double finger_joint_angle);

  rclcpp::Node::SharedPtr nh_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::string frame_to_publish_;
  double joint_vel_cmd_;
  double gripper_speed_multiplier_;
  double last_finger_joint_angle_;
  double gripper_lower_limit_;
  double gripper_upper_limit_;
};

KeyboardServo::KeyboardServo() {

    // Create the node
    nh_ = rclcpp::Node::make_shared("keyboard_servo");

    // Declare the parameters
    nh_->declare_parameter("joint_vel_cmd", 1.0);
    nh_->declare_parameter("gripper_speed_multiplier", 0.1);
    nh_->declare_parameter("gripper_lower_limit", 0.1);
    nh_->declare_parameter("gripper_upper_limit", 0.4);

    // Get the parameters from the node
    nh_->get_parameter("joint_vel_cmd", joint_vel_cmd_);
    nh_->get_parameter("gripper_speed_multiplier", gripper_speed_multiplier_);
    nh_->get_parameter("gripper_lower_limit", gripper_lower_limit_);
    nh_->get_parameter("gripper_upper_limit", gripper_upper_limit_);

    // Print the parameters
    printf("Joint velocity command: %f\n", joint_vel_cmd_);
    printf("Gripper speed multiplier: %f\n", gripper_speed_multiplier_);
    printf("Gripper lower limit: %f\n", gripper_lower_limit_);
    printf("Gripper upper limit: %f\n", gripper_upper_limit_);

    twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
    joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
    gripper_cmd_pub_ = nh_->create_publisher<std_msgs::msg::Float64MultiArray>(GRIPPER_TOPIC, ROS_QUEUE_SIZE);
    joint_state_sub_ = nh_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", ROS_QUEUE_SIZE,
    std::bind(&KeyboardServo::jointStateCallback, this, std::placeholders::_1));

}


KeyboardReader input;

// Implement the jointStateCallback function
void KeyboardServo::jointStateCallback(sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Find the index of the finger joint in the message
  auto it = std::find(msg->name.begin(), msg->name.end(), "finger_joint");
  if (it != msg->name.end())
  {
    size_t index = std::distance(msg->name.begin(), it);
    // Set the last finger joint angle based on the received message
    last_finger_joint_angle_ = msg->position[index];
  }
}

void KeyboardServo::publishGripperCommand(double finger_joint_angle)
{
  auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
  msg->data.push_back(finger_joint_angle);
  gripper_cmd_pub_->publish(std::move(msg));
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

void KeyboardServo::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

void KeyboardServo::handlePlusPress()
{
    // Calculate the new finger joint angle
    double delta_angle = 1.0 * gripper_speed_multiplier_;
    double new_finger_joint_angle = last_finger_joint_angle_ + delta_angle;
//    printf("New finger joint angle: %f\n", new_finger_joint_angle);
    // Check if the new angle is within the limits
    if (new_finger_joint_angle <= gripper_upper_limit_)
    {
        // Update the finger joint angle
        last_finger_joint_angle_ = new_finger_joint_angle;
//        printf("New finger joint angle: %f\n", last_finger_joint_angle_);
        // Publish the gripper command with the new angle
        publishGripperCommand(last_finger_joint_angle_);
    }
}

void KeyboardServo::handleMinusPress()
{
    // Calculate the new finger joint angle
    double delta_angle = -1.0 * gripper_speed_multiplier_;
    double new_finger_joint_angle = last_finger_joint_angle_ + delta_angle;

    // Check if the new angle is within the limits
    if (new_finger_joint_angle >= gripper_lower_limit_)
    {
        // Update the finger joint angle
        last_finger_joint_angle_ = new_finger_joint_angle;

        // Publish the gripper command with the new angle
        publishGripperCommand(last_finger_joint_angle_);
    }
}



int KeyboardServo::keyLoop()
{
  char c;
  bool publish_twist = false;
  bool publish_joint = false;

  std::thread{ std::bind(&KeyboardServo::spin, this) }.detach();

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
    puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
    puts("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.");
    puts("Use '+' to open the gripper, '-' to close it.");
    puts("'Q' to quit.");


  for (;;)
  {
    // get the next event from the keyboard
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error&)
    {
      perror("read():");
      return -1;
    }

    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

    // // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // Use read key-press
    switch (c)
    {
      case KEYCODE_LEFT:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        twist_msg->twist.linear.y = -1.0;
        publish_twist = true;
        break;
      case KEYCODE_RIGHT:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        twist_msg->twist.linear.y = 1.0;
        publish_twist = true;
        break;
      case KEYCODE_UP:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        twist_msg->twist.linear.x = 1.0;
        publish_twist = true;
        break;
      case KEYCODE_DOWN:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        twist_msg->twist.linear.x = -1.0;
        publish_twist = true;
        break;
      case KEYCODE_PERIOD:
        RCLCPP_DEBUG(nh_->get_logger(), "PERIOD");
        twist_msg->twist.linear.z = -1.0;
        publish_twist = true;
        break;
      case KEYCODE_SEMICOLON:
        RCLCPP_DEBUG(nh_->get_logger(), "SEMICOLON");
        twist_msg->twist.linear.z = 1.0;
        publish_twist = true;
        break;
      case KEYCODE_E:
        RCLCPP_DEBUG(nh_->get_logger(), "E");
        frame_to_publish_ = EEF_FRAME_ID;
        break;
      case KEYCODE_W:
        RCLCPP_DEBUG(nh_->get_logger(), "W");
        frame_to_publish_ = BASE_FRAME_ID;
        break;
      case KEYCODE_1:
        RCLCPP_DEBUG(nh_->get_logger(), "1");
        joint_msg->joint_names.push_back("shoulder_lift_joint");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_2:
        RCLCPP_DEBUG(nh_->get_logger(), "2");
        joint_msg->joint_names.push_back("shoulder_pan_joint");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_3:
        RCLCPP_DEBUG(nh_->get_logger(), "3");
        joint_msg->joint_names.push_back("elbow_joint");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_4:
        RCLCPP_DEBUG(nh_->get_logger(), "4");
        joint_msg->joint_names.push_back("wrist_1_joint");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_5:
        RCLCPP_DEBUG(nh_->get_logger(), "5");
        joint_msg->joint_names.push_back("wrist_2_joint");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_6:
        RCLCPP_DEBUG(nh_->get_logger(), "6");
        joint_msg->joint_names.push_back("wrist_3_joint");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
//      case KEYCODE_7:
//        RCLCPP_DEBUG(nh_->get_logger(), "7");
//        joint_msg->joint_names.push_back("finger_joint");
//        joint_msg->velocities.push_back(joint_vel_cmd_);
//        publish_joint = true;
//        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(nh_->get_logger(), "R");
        joint_vel_cmd_ *= -1;
        break;
      case KEYCODE_Q:
        RCLCPP_DEBUG(nh_->get_logger(), "quit");
        return 0;
      // Add cases for other keys as needed
      case KEYCODE_PLUS:
        RCLCPP_DEBUG(nh_->get_logger(), "PLUS");
        handlePlusPress();
        break;
      case KEYCODE_MINUS:
        RCLCPP_DEBUG(nh_->get_logger(), "MINUS");
        handleMinusPress();
        break;
    }

    // If a key requiring a publish was pressed, publish the message now
    if (publish_twist)
    {
      twist_msg->header.stamp = nh_->now();
      twist_msg->header.frame_id = frame_to_publish_;
      twist_pub_->publish(std::move(twist_msg));
      publish_twist = false;
    }
    else if (publish_joint)
    {
      joint_msg->header.stamp = nh_->now();
      joint_msg->header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(std::move(joint_msg));
      publish_joint = false;
    }
  }

  return 0;
}