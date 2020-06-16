// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the PI-Hexapod Driver.
//
//
// © Copyright 2020 Physik Instrumente (PI) GmbH & Co. KG, Karlsruhe, Germany
// © Copyright 2020 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * This file contains the implementation of InteractiveMarkerNode and a main() to be run as Node.
 *
 * \author  Christian Eichmann <eichmann@fzi.de>
 * \author  Philip Keller <keller@fzi.de>
 * \date    2020-05-06
 *
 */
//----------------------------------------------------------------------

#include "Eigen/Geometry"
#include <pi_hexapod_control/interactive_marker_node.h>

namespace pi_hexapod_control {

InteractiveMarkerNode::InteractiveMarkerNode()
  : joint_state_received_(false){};

void InteractiveMarkerNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_state_received_ = true;
  for (size_t i = 0; i < msg->name.size(); i++)
  {
    if (msg->name[i] == "ang_u")
    {
      received_joint_state_angles_[0] = msg->position[i];
    }
    else if (msg->name[i] == "ang_v")
    {
      received_joint_state_angles_[1] = msg->position[i];
    }
    else if (msg->name[i] == "ang_w")
    {
      received_joint_state_angles_[2] = msg->position[i];
    }
    else if (msg->name[i] == "cart_x")
    {
      received_joint_state_translation_[0] = msg->position[i];
    }
    else if (msg->name[i] == "cart_y")
    {
      received_joint_state_translation_[1] = msg->position[i];
    }
    else if (msg->name[i] == "cart_z")
    {
      received_joint_state_translation_[2] = msg->position[i];
    }
  }
}

bool InteractiveMarkerNode::activateCommandPublishingCallback(std_srvs::SetBool::Request& req,
                                                              std_srvs::SetBool::Response& res)
{
  setPublishingCommands(req.data);
  res.success = true;
  return true;
}

bool InteractiveMarkerNode::setToZeroCallback(std_srvs::Empty::Request& req,
                                              std_srvs::Empty::Response& res)
{
  Eigen::Array3d zeros = Eigen::Array3d::Zero();
  setToPosition(zeros, zeros);
  return true;
}

bool InteractiveMarkerNode::setToCurrentCallback(std_srvs::Empty::Request& req,
                                                 std_srvs::Empty::Response& res)
{
  if (joint_state_received_)
  {
    setToPosition(received_joint_state_translation_, received_joint_state_angles_);
  }
  else
  {
    ROS_ERROR("Can't set marker to current position. Interactive Marker Server did not receive "
              "joint_states.");
  }
  return true;
}

bool InteractiveMarkerNode::sendSingleCommandCallback(std_srvs::Empty::Request& req,
                                                      std_srvs::Empty::Response& res)
{
  publishCommand();
  return true;
}

void InteractiveMarkerNode::setToPosition(const Eigen::Array3d& translation,
                                          const Eigen::Array3d& angles)
{
  ROS_INFO("Setting marker to position.");

  // set rotation axis controller
  for (size_t i = 0; i < 3; i++)
  {
    Eigen::Vector3d axis;
    axis[i] = 1;
    Eigen::AngleAxis<double> aa(angles[i], axis);
    Eigen::Quaternion<double> quaternion(aa);
    geometry_msgs::Pose axis_pose;
    axis_pose.orientation.w = quaternion.w();
    axis_pose.orientation.x = quaternion.x();
    axis_pose.orientation.y = quaternion.y();
    axis_pose.orientation.z = quaternion.z();
    server_->setPose(MAKER_NAME_FROM_AXIS[i], axis_pose);
    // server_->applyChanges() is called from updatePoseMsg()
  }

  // set marker position
  translation_ = translation;
  angles_      = angles;
  updatePoseMsg();
}

void InteractiveMarkerNode::menuPublishCommandsHandleCallback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  interactive_markers::MenuHandler::CheckState state;
  menu_handler_.getCheckState(menu_publish_commands_handle_, state);

  // if previously unchecked, activate now and vice versa
  setPublishingCommands(state == interactive_markers::MenuHandler::UNCHECKED);
}

void InteractiveMarkerNode::setPublishingCommands(const bool new_state)
{
  is_publishing_commands_ = new_state;
  ROS_INFO("Command publication %s.", (is_publishing_commands_ ? "activated" : "deactivated"));

  if (is_publishing_commands_)
  {
    menu_handler_.setCheckState(menu_publish_commands_handle_,
                                interactive_markers::MenuHandler::CHECKED);
  }
  else
  {
    menu_handler_.setCheckState(menu_publish_commands_handle_,
                                interactive_markers::MenuHandler::UNCHECKED);
  }
  menu_handler_.reApply(*server_.get());
  server_->applyChanges();
}


void InteractiveMarkerNode::translationMarkerCallback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  Eigen::Array3d vector(
    feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

  translation_ = lower_translation_limits_.max(upper_translation_limits_.min(vector));

  updatePoseMsg();
}

void InteractiveMarkerNode::rotationMarkerCallback(
  const int32_t axis_id, const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  geometry_msgs::Pose pose = feedback->pose;

  const Eigen::Quaternion<double> feedback_quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const Eigen::AngleAxis<double> feedback_angle_axis(feedback_quaternion);

  // determine algebraic sign of angle from rotation axis
  const double angle = feedback_angle_axis.angle() * feedback_angle_axis.axis()[axis_id];
  angles_[axis_id] =
    std::max(lower_angle_limits_[axis_id], std::min(upper_angle_limits_[axis_id], angle));

  const Eigen::AngleAxis<double> limited_angle_axis(
    angles_[axis_id] * feedback_angle_axis.axis()[axis_id], feedback_angle_axis.axis());
  const Eigen::Quaternion<double> limited_quaternion(limited_angle_axis);
  pose.orientation.w = limited_quaternion.w();
  pose.orientation.x = limited_quaternion.x();
  pose.orientation.y = limited_quaternion.y();
  pose.orientation.z = limited_quaternion.z();

  server_->setPose(MAKER_NAME_FROM_AXIS[axis_id], pose);
  updatePoseMsg();
}

void InteractiveMarkerNode::updatePoseMsg()
{
  const Eigen::Quaternion<double> q =
    Eigen::AngleAxisd(angles_[ROLL_AXIS_ID], Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(angles_[PITCH_AXIS_ID], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(angles_[YAW_AXIS_ID], Eigen::Vector3d::UnitZ());

  pose_msg_.pose.position.x    = translation_[0];
  pose_msg_.pose.position.y    = translation_[1];
  pose_msg_.pose.position.z    = translation_[2];
  pose_msg_.pose.orientation.w = q.w();
  pose_msg_.pose.orientation.x = q.x();
  pose_msg_.pose.orientation.y = q.y();
  pose_msg_.pose.orientation.z = q.z();

  server_->setPose(TRANSLATION_MARKER, pose_msg_.pose);
  server_->setPose(VISUALIZATION_MARKER, pose_msg_.pose);
  server_->applyChanges();
}

void InteractiveMarkerNode::initMenu()
{
  menu_handler_.insert(
    "Set to zero",
    std::bind(&InteractiveMarkerNode::setToZeroMenuCallback, this, std::placeholders::_1));

  menu_handler_.insert(
    "Set to current",
    std::bind(&InteractiveMarkerNode::setToCurrentMenuCallback, this, std::placeholders::_1));

  menu_handler_.insert("Send marker as command",
                       std::bind(&InteractiveMarkerNode::publishCommand, this));


  menu_publish_commands_handle_ = menu_handler_.insert(
    "Publish commands",
    std::bind(
      &InteractiveMarkerNode::menuPublishCommandsHandleCallback, this, std::placeholders::_1));

  if (is_publishing_commands_)
  {
    menu_handler_.setCheckState(menu_publish_commands_handle_,
                                interactive_markers::MenuHandler::CHECKED);
  }
  else
  {
    menu_handler_.setCheckState(menu_publish_commands_handle_,
                                interactive_markers::MenuHandler::UNCHECKED);
  }
}

void InteractiveMarkerNode::setToZeroMenuCallback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  Eigen::Array3d zeros = Eigen::Array3d::Zero();
  setToPosition(zeros, zeros);
}

void InteractiveMarkerNode::setToCurrentMenuCallback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  if (joint_state_received_)
  {
    setToPosition(received_joint_state_translation_, received_joint_state_angles_);
  }
  else
  {
    ROS_ERROR("Can't set marker to current position. Interactive Marker Server did not receive "
              "joint_states.");
  }
}


bool InteractiveMarkerNode::init(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
{
  private_nh.param<double>("rate", rate_, 10.0);
  private_nh.param<double>("interactive_marker/control_scale", control_scale_, 1.0);
  private_nh.param<bool>("autostart_publishing_commands", is_publishing_commands_, false);
  private_nh.param<std::string>("topic_ns", topic_ns_, "pi_interactive_marker");
  private_nh.param<std::string>("interactive_marker/base_frame", base_frame_, "zero_link");

  const double neg_max = -1 * std::numeric_limits<double>::max();
  const double pos_max = std::numeric_limits<double>::max();
  private_nh.param<double>("x/lower_limit", lower_translation_limits_[0], neg_max);
  private_nh.param<double>("x/upper_limit", upper_translation_limits_[0], pos_max);
  private_nh.param<double>("y/lower_limit", lower_translation_limits_[1], neg_max);
  private_nh.param<double>("y/upper_limit", upper_translation_limits_[1], pos_max);
  private_nh.param<double>("z/lower_limit", lower_translation_limits_[2], neg_max);
  private_nh.param<double>("z/upper_limit", upper_translation_limits_[2], pos_max);
  private_nh.param<double>("u/lower_limit", lower_angle_limits_[ROLL_AXIS_ID], neg_max);
  private_nh.param<double>("u/upper_limit", upper_angle_limits_[ROLL_AXIS_ID], pos_max);
  private_nh.param<double>("v/lower_limit", lower_angle_limits_[PITCH_AXIS_ID], neg_max);
  private_nh.param<double>("v/upper_limit", upper_angle_limits_[PITCH_AXIS_ID], pos_max);
  private_nh.param<double>("w/lower_limit", lower_angle_limits_[YAW_AXIS_ID], neg_max);
  private_nh.param<double>("w/upper_limit", upper_angle_limits_[YAW_AXIS_ID], pos_max);

  // Initialize the pose to 0, so it is a valid pose even before the first callback
  pose_msg_.pose.position.x    = 0;
  pose_msg_.pose.position.y    = 0;
  pose_msg_.pose.position.z    = 0;
  pose_msg_.pose.orientation.x = 0;
  pose_msg_.pose.orientation.y = 0;
  pose_msg_.pose.orientation.z = 0;
  pose_msg_.pose.orientation.w = 1;

  // create Marker Server but do not spin up a thread to handle callbacks
  server_.reset(new interactive_markers::InteractiveMarkerServer(topic_ns_, "", false));

  // Initialize right click menu and set its callback handlers
  initMenu();

  /* ******************* *
   * VISUALIZATION MAKER *
   * ******************* */
  visualization_msgs::InteractiveMarker visualization_marker;
  visualization_marker.header.frame_id = base_frame_;
  visualization_marker.name            = VISUALIZATION_MARKER;
  visualization_marker.scale           = control_scale_;

  // Create visualized cylinder
  visualization_msgs::InteractiveMarkerControl cylinder_control;
  cylinder_control.always_visible   = true;
  cylinder_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  visualization_msgs::Marker platform_marker;
  platform_marker.type = visualization_msgs::Marker::CYLINDER;
  private_nh.param<double>("interactive_marker/scale/x", platform_marker.scale.x, 0.2);
  private_nh.param<double>("interactive_marker/scale/y", platform_marker.scale.y, 0.2);
  private_nh.param<double>("interactive_marker/scale/z", platform_marker.scale.z, 0.1);
  private_nh.param<double>("interactive_marker/position/x", platform_marker.pose.position.x, 0.0);
  private_nh.param<double>("interactive_marker/position/y", platform_marker.pose.position.y, 0.0);
  private_nh.param<double>("interactive_marker/position/z", platform_marker.pose.position.z, 0.0);
  private_nh.param<double>(
    "interactive_marker/orientation/w", platform_marker.pose.orientation.w, 1.0);
  private_nh.param<double>(
    "interactive_marker/orientation/x", platform_marker.pose.orientation.x, 0.0);
  private_nh.param<double>(
    "interactive_marker/orientation/y", platform_marker.pose.orientation.y, 0.0);
  private_nh.param<double>(
    "interactive_marker/orientation/z", platform_marker.pose.orientation.z, 0.0);
  private_nh.param<float>("interactive_marker/color/r", platform_marker.color.r, 1.0);
  private_nh.param<float>("interactive_marker/color/g", platform_marker.color.g, 0.0);
  private_nh.param<float>("interactive_marker/color/b", platform_marker.color.b, 0.0);
  private_nh.param<float>("interactive_marker/color/a", platform_marker.color.a, 0.5);
  cylinder_control.markers.push_back(platform_marker);

  // Add cube to form X-Axis Arrow
  visualization_msgs::Marker x_axis_marker(platform_marker);
  x_axis_marker.color.r            = 1.0;
  x_axis_marker.color.b            = 0.0;
  x_axis_marker.pose.position.x    = platform_marker.scale.x * 0.25 + 0.005;
  x_axis_marker.scale.x            = 0.005;
  x_axis_marker.scale.y            = 0.005;
  x_axis_marker.scale.z            = platform_marker.scale.x * 0.5 + 0.01;
  x_axis_marker.pose.orientation.w = 0.7071068;
  x_axis_marker.pose.orientation.y = 0.7071068;
  cylinder_control.markers.push_back(x_axis_marker);

  // Add cube to form Y-Axis Arrow
  visualization_msgs::Marker y_axis_marker(platform_marker);
  y_axis_marker.color.g            = 1.0;
  y_axis_marker.color.b            = 0.0;
  y_axis_marker.pose.position.x    = 0;
  y_axis_marker.pose.position.y    = platform_marker.scale.y * 0.25 + 0.005;
  y_axis_marker.scale.x            = 0.005;
  y_axis_marker.scale.y            = 0.005;
  y_axis_marker.scale.z            = platform_marker.scale.y * 0.5 + 0.01;
  y_axis_marker.pose.orientation.w = 0.7071068;
  y_axis_marker.pose.orientation.x = 0.7071068;
  cylinder_control.markers.push_back(y_axis_marker);

  visualization_marker.controls.push_back(cylinder_control);
  server_->insert(visualization_marker);
  menu_handler_.apply(*server_.get(), visualization_marker.name);

  /* ***************** *
   * TRANSLATION MAKER *
   * ***************** */
  visualization_msgs::InteractiveMarker translation_marker;
  translation_marker.header.frame_id = base_frame_;
  translation_marker.name            = TRANSLATION_MARKER;
  translation_marker.scale           = control_scale_;

  // Add three different interaction controls to Marker
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w    = 0.7071068;
  control.orientation.x    = 0.7071068;
  control.orientation.y    = 0;
  control.orientation.z    = 0;
  control.name             = "move_x";
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  translation_marker.controls.push_back(control);

  control.orientation.w    = 0.7071068;
  control.orientation.x    = 0;
  control.orientation.y    = 0;
  control.orientation.z    = 0.7071068;
  control.name             = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  translation_marker.controls.push_back(control);

  control.orientation.w    = 0.7071068;
  control.orientation.x    = 0;
  control.orientation.y    = 0.7071068;
  control.orientation.z    = 0;
  control.name             = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  translation_marker.controls.push_back(control);

  server_->insert(translation_marker);
  menu_handler_.apply(*server_.get(), translation_marker.name);
  server_->setCallback(
    translation_marker.name,
    std::bind(&InteractiveMarkerNode::translationMarkerCallback, this, std::placeholders::_1));

  /* *********** *
   * ROLL MARKER *
   * *********** */
  visualization_msgs::InteractiveMarker roll_marker;
  roll_marker.header.frame_id = base_frame_;
  roll_marker.name            = MAKER_NAME_FROM_AXIS[ROLL_AXIS_ID];
  roll_marker.scale           = control_scale_;
  control.orientation.w       = 0.7071068;
  control.orientation.x       = 0.7071068;
  control.orientation.y       = 0;
  control.orientation.z       = 0;
  control.name                = "rotate_u";
  control.interaction_mode    = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  roll_marker.controls.push_back(control);
  server_->insert(roll_marker);
  menu_handler_.apply(*server_.get(), roll_marker.name);
  server_->setCallback(
    roll_marker.name,
    std::bind(
      &InteractiveMarkerNode::rotationMarkerCallback, this, ROLL_AXIS_ID, std::placeholders::_1));

  /* ************ *
   * PITCH MARKER *
   * ************ */
  visualization_msgs::InteractiveMarker pitch_marker;
  pitch_marker.header.frame_id = base_frame_;
  pitch_marker.name            = MAKER_NAME_FROM_AXIS[PITCH_AXIS_ID];
  pitch_marker.scale           = control_scale_;
  control.orientation.w        = 0.7071068;
  control.orientation.x        = 0;
  control.orientation.y        = 0;
  control.orientation.z        = 0.7071068;
  control.name                 = "rotate_v";
  control.interaction_mode     = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  pitch_marker.controls.push_back(control);
  server_->insert(pitch_marker);
  menu_handler_.apply(*server_.get(), pitch_marker.name);
  server_->setCallback(
    pitch_marker.name,
    std::bind(
      &InteractiveMarkerNode::rotationMarkerCallback, this, PITCH_AXIS_ID, std::placeholders::_1));

  /* ********** *
   * YAW MARKER *
   * ********** */
  visualization_msgs::InteractiveMarker yaw_marker;
  yaw_marker.header.frame_id = base_frame_;
  yaw_marker.name            = MAKER_NAME_FROM_AXIS[YAW_AXIS_ID];
  yaw_marker.scale           = control_scale_;
  control.orientation.w      = 0.7071068;
  control.orientation.x      = 0;
  control.orientation.y      = 0.7071068;
  control.orientation.z      = 0;
  control.name               = "rotate_w";
  control.interaction_mode   = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  yaw_marker.controls.push_back(control);
  server_->insert(yaw_marker);
  menu_handler_.apply(*server_.get(), yaw_marker.name);
  server_->setCallback(
    yaw_marker.name,
    std::bind(
      &InteractiveMarkerNode::rotationMarkerCallback, this, YAW_AXIS_ID, std::placeholders::_1));

  // Apply changes to server
  server_->applyChanges();

  // Setup publisher to publish pose in relation to the base frame
  pose_publisher_ = private_nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
  // Setup publisher to publish command to joint_group_position_controller
  command_publisher_ = private_nh.advertise<std_msgs::Float64MultiArray>("command", 10);
  // Setup joint state subscriber
  joint_state_subscriber_ = private_nh.subscribe<sensor_msgs::JointState>(
    "joint_states", 100, &InteractiveMarkerNode::jointStateCallback, this);

  activate_command_publishing_server_ = private_nh.advertiseService(
    "activate_command_publishing", &InteractiveMarkerNode::activateCommandPublishingCallback, this);
  set_to_zero_server_ =
    private_nh.advertiseService("set_to_zero", &InteractiveMarkerNode::setToZeroCallback, this);
  set_to_current_server_ = private_nh.advertiseService(
    "set_to_current", &InteractiveMarkerNode::setToCurrentCallback, this);
  send_single_command_server_ = private_nh.advertiseService(
    "send_single_command", &InteractiveMarkerNode::sendSingleCommandCallback, this);

  // Setup the timer which is used to send out messages with a given rate
  ros::Duration period(1.0 / rate_);
  timer_ = nh.createTimer(
    period, std::bind(&InteractiveMarkerNode::timerCallback, this, std::placeholders::_1));

  return true;
}

void InteractiveMarkerNode::timerCallback(const ros::TimerEvent& timer_event)
{
  pose_msg_.header.frame_id = base_frame_;
  pose_msg_.header.stamp    = ros::Time::now();
  pose_publisher_.publish(pose_msg_);

  if (is_publishing_commands_)
  {
    publishCommand();
  }
}

void InteractiveMarkerNode::publishCommand()
{
  std_msgs::Float64MultiArray command;
  command.data.push_back(translation_[0]);
  command.data.push_back(translation_[1]);
  command.data.push_back(translation_[2]);
  command.data.push_back(angles_[0]);
  command.data.push_back(angles_[1]);
  command.data.push_back(angles_[2]);
  command_publisher_.publish(command);
}

} // namespace pi_hexapod_control

/*!
 * \brief Launches a InteractiveMarkerNode as ROS Node.
 *
 * \param argc
 * \param argv
 * \return int
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_marker_tf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pi_hexapod_control::InteractiveMarkerNode node = pi_hexapod_control::InteractiveMarkerNode();

  if (!node.init(nh, pnh))
  {
    ROS_ERROR("Setup failed, shutting down");
    return -1;
  }

  ROS_INFO("Start spinning.");
  ros::spin();
  ROS_INFO("Stopped spinning, shutting down.");

  return 0;
}
