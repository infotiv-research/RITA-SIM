#include "curobo_rviz/arrow_interaction.hpp"

#include <chrono>

#include <tf2/exceptions.h>
#include <tf2/time.h>

ArrowInteraction::ArrowInteraction(std::shared_ptr<rclcpp::Node> node)
: node_(node),
  double_click_threshold_(0.3),
  frame_id_("base_0"),
  is_visible_(true),
  initialization_source_frame_(""),
  pose_initialized_from_tf_(false)
{
  // Create the interactive marker server using the provided node.
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>( "simple_marker", node_.get());
  RCLCPP_INFO(node_->get_logger(), "ArrowInteraction: Interactive marker server created");

  // Create a publisher to publish marker poses.
  pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("marker_pose", 10);

  // Initialize the last click time.
  last_click_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
  initial_pose_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&ArrowInteraction::attemptInitialPoseFromTf, this));

  // Initialize marker pose to identity
  marker_pose_.position.x = 0.0;
  marker_pose_.position.y = 0.0;
  marker_pose_.position.z = 0.0;
  marker_pose_.orientation.x = 0.0;
  marker_pose_.orientation.y = 0.0;
  marker_pose_.orientation.z = 0.0;
  marker_pose_.orientation.w = 1.0;

  // Create the 6-DOF marker with an initial position (0,0,0).
  geometry_msgs::msg::Point position;
  position.x = 0.0;
  position.y = 0.0;
  position.z = 0.0;
  make6DofMarker(position);
  RCLCPP_INFO(node_->get_logger(), "ArrowInteraction: Constructor completed, marker should be visible");
}

ArrowInteraction::~ArrowInteraction()
{
  if (server_) {
    server_->clear();
  }
}

void ArrowInteraction::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    // Get the current pose from the feedback.
    marker_pose_ = feedback->pose;
    pose_initialized_from_tf_ = true;
  }

}

void ArrowInteraction::make6DofMarker(const geometry_msgs::msg::Point & position)
{
  RCLCPP_INFO(node_->get_logger(), "ArrowInteraction: Creating marker in frame '%s' at position (%.2f, %.2f, %.2f)",
              frame_id_.c_str(), position.x, position.y, position.z);

  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id_;
  int_marker.header.stamp = node_->get_clock()->now();
  int_marker.pose.position = position;
  int_marker.pose.orientation = marker_pose_.orientation;
  int_marker.scale = 1.0;
  int_marker.name = "simple_6dof";
  int_marker.description = "Simple 6-DOF Control";

  auto make_box_marker = [](
      int id,
      double x,
      double y,
      double z,
      double sx,
      double sy,
      double sz,
      double qx,
      double qy,
      double qz,
      double qw,
      double r,
      double g,
      double b,
      double a) {
    visualization_msgs::msg::Marker marker;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = qx;
    marker.pose.orientation.y = qy;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;
    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
  };

  // Stylized Robotiq 2F-140 gripper around TCP_point for easier pose reading.
  // The geometry is expressed directly in TCP_point using the URDF's actual
  // TCP-to-pad layout, instead of applying an extra guessed frame rotation.
  constexpr double qx = 0.0;
  constexpr double qy = 0.0;
  constexpr double qz = 0.0;
  constexpr double qw = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl gripper_control;
  gripper_control.always_visible = true;
  auto add_gripper_box = [&](int id,
                             double x,
                             double y,
                             double z,
                             double sx,
                             double sy,
                             double sz,
                             double r,
                             double g,
                             double b,
                             double a) {
    gripper_control.markers.push_back(make_box_marker(
        id, x, y, z, sx, sy, sz, qx, qy, qz, qw, r, g, b, a));
  };
  add_gripper_box(1, 0.0, 0.0, -0.145, 0.080, 0.080, 0.095, 0.25, 0.25, 0.28, 0.92);
  add_gripper_box(3, 0.052, 0.0, -0.082, 0.020, 0.020, 0.085, 0.10, 0.10, 0.12, 0.96);
  add_gripper_box(4, -0.052, 0.0, -0.082, 0.020, 0.020, 0.085, 0.10, 0.10, 0.12, 0.96);
  add_gripper_box(5, 0.068, 0.0, -0.033, 0.0075, 0.027, 0.065, 0.85, 0.85, 0.88, 0.98);
  add_gripper_box(6, -0.068, 0.0, -0.033, 0.0075, 0.027, 0.065, 0.85, 0.85, 0.88, 0.98);
  int_marker.controls.push_back(gripper_control);

  // Add 6-DOF controls (rotate and move for X, Y, and Z axes).
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = 1.0;
    control.orientation.y = 0.0;
    control.orientation.z = 0.0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
  }
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = 1.0;
    control.orientation.y = 0.0;
    control.orientation.z = 0.0;
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 1.0;
    control.orientation.z = 0.0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
  }
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 1.0;
    control.orientation.z = 0.0;
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 0.0;
    control.orientation.z = 1.0;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
  }
  {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = 0.0;
    control.orientation.y = 0.0;
    control.orientation.z = 1.0;
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  // Insert the marker into the server and bind the feedback callback.
  server_->insert(int_marker, std::bind(&ArrowInteraction::processFeedback, this, std::placeholders::_1));
  server_->applyChanges();

  RCLCPP_INFO(node_->get_logger(), "ArrowInteraction: Marker inserted and changes applied. Marker should now be visible.");
  RCLCPP_INFO(node_->get_logger(), "ArrowInteraction: Check topic /simple_marker/update for interactive marker messages");
}

geometry_msgs::msg::Pose ArrowInteraction::get_pose(){
    return this->marker_pose_;
}

void ArrowInteraction::setFrameId(const std::string& frame_id) {
  frame_id_ = frame_id;
  pose_initialized_from_tf_ = initialization_source_frame_.empty();

  // Recreate the marker with the new frame_id
  // Preserve the current position
  geometry_msgs::msg::Point position;
  position.x = marker_pose_.position.x;
  position.y = marker_pose_.position.y;
  position.z = marker_pose_.position.z;

  // Clear existing marker
  server_->clear();

  // Recreate with new frame_id
  make6DofMarker(position);
}

std::string ArrowInteraction::getFrameId() const {
  return frame_id_;
}

void ArrowInteraction::resetPose() {
  // Reset to origin
  geometry_msgs::msg::Point position;
  position.x = 0.0;
  position.y = 0.0;
  position.z = 0.0;

  // Reset the stored pose
  marker_pose_.position = position;
  marker_pose_.orientation.x = 0.0;
  marker_pose_.orientation.y = 0.0;
  marker_pose_.orientation.z = 0.0;
  marker_pose_.orientation.w = 1.0;
  pose_initialized_from_tf_ = true;

  // Recreate the marker at origin
  server_->clear();
  make6DofMarker(position);
}

void ArrowInteraction::setPose(const geometry_msgs::msg::Point& position) {
  // Update the stored pose
  marker_pose_.position = position;
  pose_initialized_from_tf_ = true;

  // Recreate the marker at the new position
  server_->clear();
  make6DofMarker(position);

  RCLCPP_INFO(node_->get_logger(), "ArrowInteraction: Marker pose updated to (%.3f, %.3f, %.3f)",
              position.x, position.y, position.z);
}

void ArrowInteraction::setPoseWithOrientation(const geometry_msgs::msg::Pose& pose) {
  // Update the stored pose completely (position and orientation)
  marker_pose_ = pose;
  pose_initialized_from_tf_ = true;
  server_->clear();
  make6DofMarker(pose.position);
  RCLCPP_INFO(node_->get_logger(), "ArrowInteraction: Marker pose with orientation updated");
}

void ArrowInteraction::setVisible(bool visible) {
  is_visible_ = visible;
  RCLCPP_INFO(node_->get_logger(), "ArrowInteraction: setVisible called with value: %s", visible ? "true" : "false");

  if (!visible) {
    // Clear the marker to hide it
    server_->clear();
    server_->applyChanges();
    RCLCPP_INFO(node_->get_logger(), "ArrowInteraction: Marker hidden");
  } else {
    // Recreate the marker to show it
    geometry_msgs::msg::Point position;
    position.x = marker_pose_.position.x;
    position.y = marker_pose_.position.y;
    position.z = marker_pose_.position.z;
    make6DofMarker(position);
    RCLCPP_INFO(node_->get_logger(), "ArrowInteraction: Marker shown");
  }
}

void ArrowInteraction::setInitializationSourceFrame(const std::string& source_frame) {
  initialization_source_frame_ = source_frame;
  pose_initialized_from_tf_ = initialization_source_frame_.empty();
}

void ArrowInteraction::attemptInitialPoseFromTf() {
  if (pose_initialized_from_tf_ || initialization_source_frame_.empty()) {
    return;
  }

  try {
    const auto transform = tf_buffer_->lookupTransform(
      frame_id_, initialization_source_frame_, tf2::TimePointZero);

    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    marker_pose_ = pose;
    pose_initialized_from_tf_ = true;

    if (is_visible_) {
      server_->clear();
      make6DofMarker(pose.position);
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "ArrowInteraction: Initialized marker pose from TF %s -> %s",
      frame_id_.c_str(),
      initialization_source_frame_.c_str());
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      2000,
      "ArrowInteraction: waiting for TF %s -> %s: %s",
      frame_id_.c_str(),
      initialization_source_frame_.c_str(),
      ex.what());
  }
}
