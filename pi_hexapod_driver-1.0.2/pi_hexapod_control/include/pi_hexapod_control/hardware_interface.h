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
 * This file contains the HardwareInterface (hardware_interface::RobotHW) class for PI Hexapods.
 *
 * \author  Christian Eichmann <eichmann@fzi.de>
 * \author  Philip Keller <keller@fzi.de>
 * \date    2020-04-06
 *
 */
//----------------------------------------------------------------------
#ifndef PI_HEXAPOD_CONTROL_HARDWARE_INTERFACE_H_INCLUDED
#define PI_HEXAPOD_CONTROL_HARDWARE_INTERFACE_H_INCLUDED


#include <actionlib/server/simple_action_server.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pi_hexapod_control/visual_joint_generator.h>
#include <pi_hexapod_msgs/ReferencingAction.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include "pi_hexapod_control/pi_driver.h"

namespace pi_hexapod_control {

/*!
 * \brief Hardware Interface to control a Physik Instrumente (PI) Hexapod
 */
class HardwareInterface : public hardware_interface::RobotHW
{
public:
  /*!
   * \brief Creates a new HardwareInterface object.
   */
  HardwareInterface();
  virtual ~HardwareInterface() = default;

  /*!
   * \brief Initializes HardwareInterface, reads ROS params
   *
   * \param root_nh Root level ROS node handle
   * \param robot_hw_nh ROS node handle for the robot namespace
   *
   * \returns
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

  /*!
   * \brief Reads joint values from Hexapod
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void read(const ros::Time& time, const ros::Duration& period) override;

  /*!
   * \brief Sends command joint values to Hexapod
   *
   * \param time Current time
   * \param period Duration of current control loop iteration
   */
  virtual void write(const ros::Time& time, const ros::Duration& period) override;

  /*!
   * \brief Checks whether a proposed controller switch is feasable
   *
   * \param start_list List of controllers to start
   * \param stop_list List of controllers to stop
   *
   * \returns
   */
  virtual bool
  prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list) override;
  /*!
   * \brief Switches Controllers
   *
   * \param start_list List of controllers to start
   * \param stop_list List of controllers to stop
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  /*!
   * \brief Sends HALT command to Hexapod Controller.
   *
   * \returns True on successful request
   */
  bool haltHexapod();

  /*!
   * \brief Sends STOP command to Hexapod Controller.
   *
   * \returns True on successful request
   */
  bool stopHexapod();

  /*!
   * \brief Disconnects the Hexapod
   */
  void disconnectHexapod();

  /*!
   * \brief Callback to enable/disable active control of the Hexapod via ROS Service
   *
   * \param req SetBool request with the desired control mode
   * \param res True on successful request
   *
   * \returns True when callback finished
   */
  bool enableServiceCallback(std_srvs::SetBoolRequest& req, std_srvs::SetBoolResponse& res);

  /*!
   * \brief Callback to "halt" the Hexapod via ROS Service
   *
   * \param req Trigger request
   * \param res True on successful request
   *
   * \returns True when callback finished
   */
  bool haltServiceCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

  /*!
   * \brief Callback to "stop" the Hexapod via ROS Service
   *
   * \param req Trigger request
   * \param res True on successful request
   *
   * \returns True when callback finished
   */
  bool stopServiceCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

protected:
  /*!
   * \brief Callback to reference Hexapod via ROS Action
   *
   * \param goal An action goal message
   */
  void referencingActionCallback(const pi_hexapod_msgs::ReferencingGoalConstPtr& goal);

  /*!
   * \brief Publishes the ID and Message belonging to the current error of the Driver
   *
   */
  void publishDriverStatus();

private:
  std::unique_ptr<PiDriver> pi_driver_;

  std::unique_ptr<actionlib::SimpleActionServer<pi_hexapod_msgs::ReferencingAction> >
    referencing_action_server_;
  ros::ServiceServer enable_srv_;
  ros::ServiceServer halt_srv_;
  ros::ServiceServer stop_srv_;

  ros::Publisher pub_error_id_;
  ros::Publisher pub_error_msg_;

  hardware_interface::JointStateInterface js_interface_;
  hardware_interface::PositionJointInterface pj_interface_;

  std::vector<std::string> joint_names_;
  vector6d_t joint_position_command_;
  vector6d_t joint_positions_;
  vector6d_t joint_velocities_;
  vector6d_t joint_efforts_;

  std::vector<std::string> visual_joint_names_;
  vector36d_t visual_joint_positions_;
  vector36d_t visual_joint_velocities_;
  vector36d_t visual_joint_efforts_;


  volatile bool control_mode_enabled_;
  volatile bool position_controller_running_;
  bool referencing_running_;
  bool prefer_stop_over_halt_;
  int latest_error_id_;

  std::string hexapod_ip_;
  std::string tf_prefix_;

  VisualJointGenerator visual_joint_generator_;
};

} // namespace pi_hexapod_control

#endif // ifndef PI_HEXAPOD_CONTROL_HARDWARE_INTERFACE_H_INCLUDED
