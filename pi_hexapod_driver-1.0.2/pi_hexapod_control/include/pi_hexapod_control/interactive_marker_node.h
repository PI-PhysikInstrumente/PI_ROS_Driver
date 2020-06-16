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
 * This file contains the InteractiveMarkerNode which provides an interactive marker server
 * to control PI Hexapods.
 *
 * \author  Christian Eichmann <eichmann@fzi.de>
 * \author  Philip Keller <keller@fzi.de>
 * \date    2020-05-06
 *
 */
//----------------------------------------------------------------------
#ifndef PI_HEXAPOD_CONTROL_INTERACTIVE_MARKER_NODE_H_INCLUDED
#define PI_HEXAPOD_CONTROL_INTERACTIVE_MARKER_NODE_H_INCLUDED

#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

namespace pi_hexapod_control {

/**
 * \brief Node providing an Interactive Marker to control a hexapod.
 *
 * Provides an Interactive Marker to control a PI hexapod using RViz.
 * Uses a JointGroupPositionController to send commands.
 * Requires a hexapod configuration (YAML) file.
 */
class InteractiveMarkerNode
{
public:
  InteractiveMarkerNode();
  ~InteractiveMarkerNode() = default;

  /**
   * \brief Callback to receive joint state messages
   *
   * \param msg a joint state message
   */
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  /**
   * \brief initializes the Node.
   *
   * Reads configuration from the parameter server and sets up the marker server, publishers
   * and callbacks.
   *
   * \param nh Root NodeHandle
   * \param private_nh NodeHandle in the nodes namespace
   *
   * \return indicates success
   */
  bool init(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

  /**
   * \brief Handles marker updates from translations
   *
   * \param feedback new marker attributes
   *
   */
  void
  translationMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * \brief Handles marker updates from rotations
   *
   * \param axis_id ID of the axis which was rotated
   * \param feedback new marker attributes
   */
  void
  rotationMarkerCallback(const int32_t axis_id,
                         const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /**
   * \brief Sends commands to Hexapod.
   *
   * Sends marker position to topic and commands to the Controller.
   * Timer rate can be set as parameter.
   *
   * \param timer_event
   */
  void timerCallback(const ros::TimerEvent& timer_event);

  /**
   * \brief Service callback to toggle command publication
   *
   * \param req
   * \param res
   * \return indicates success of the call itself, not of the operation
   */
  bool activateCommandPublishingCallback(std_srvs::SetBool::Request& req,
                                         std_srvs::SetBool::Response& res);

  /**
   * \brief Service callback to set marker position to zero
   *
   * \param req
   * \param res
   * \return indicates success of the call itself, not of the operation
   *
   */
  bool setToZeroCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * \brief Service callback to set marker position to current position based on joint states
   *
   * \param req
   * \param res
   * \return indicates success of the call itself, not of the operation
   */
  bool setToCurrentCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * \brief Sends current marker position as command to Hexapod Controller
   *
   * \param req
   * \param res
   * \return indicates success of the call itself, not of the operation
   */
  bool sendSingleCommandCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * \brief Set internal command publication state and update menu
   *
   * \param new_state true for command publication
   * \return indicates success of the call itself, not of the operation
   */
  void setPublishingCommands(const bool new_state);

  /**
   * \brief Set marker position to zero
   *
   *  \param translation new replacements in X, Y and Z direction
   *  \param angles new rotations around X, Y and Z axis
   */
  void setToPosition(const Eigen::Array3d& translation, const Eigen::Array3d& angles);

private:
  const int32_t ROLL_AXIS_ID  = 0;
  const int32_t PITCH_AXIS_ID = 1;
  const int32_t YAW_AXIS_ID   = 2;

  const std::string TRANSLATION_MARKER   = "translation_marker";
  const std::string VISUALIZATION_MARKER = "visualization_marker";

  // initialize with double curly braces due to bug in some GCC versions for xenial (e.g. 5.4.0)
  const std::array<std::string, 3> MAKER_NAME_FROM_AXIS = {
    {"roll_marker", "pitch_marker", "yaw_maker"}};

  void updatePoseMsg();
  void publishCommand();

  void initMenu();
  void setToZeroMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
  void
  setToCurrentMenuCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  void menuPublishCommandsHandleCallback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  interactive_markers::MenuHandler menu_handler_;

  interactive_markers::MenuHandler::EntryHandle menu_publish_commands_handle_;

  ros::Timer timer_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  ros::Publisher pose_publisher_;
  ros::Publisher command_publisher_;
  ros::ServiceServer activate_command_publishing_server_;
  ros::ServiceServer set_to_zero_server_;
  ros::ServiceServer set_to_current_server_;
  ros::ServiceServer send_single_command_server_;
  ros::Subscriber joint_state_subscriber_;


  double rate_;
  double control_scale_;
  bool joint_state_received_;
  bool is_publishing_commands_;
  std::string base_frame_;
  std::string topic_ns_;
  geometry_msgs::PoseStamped pose_msg_;

  Eigen::Array3d received_joint_state_translation_;
  Eigen::Array3d received_joint_state_angles_;

  Eigen::Array3d translation_;
  Eigen::Array3d angles_;

  Eigen::Array3d lower_translation_limits_;
  Eigen::Array3d upper_translation_limits_;
  Eigen::Array3d lower_angle_limits_;
  Eigen::Array3d upper_angle_limits_;
};

} // namespace pi_hexapod_control

#endif // ifndef PI_HEXAPOD_CONTROL_INTERACTIVE_MARKER_NODE_H_INCLUDED
