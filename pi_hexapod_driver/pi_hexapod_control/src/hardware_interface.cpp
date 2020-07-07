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
 * This file contains the implementation for the HardwareInterface (hardware_interface::RobotHW)
 * for PI Hexapods.
 *
 * \author  Christian Eichmann <eichmann@fzi.de>
 * \author  Philip Keller <keller@fzi.de>
 * \date    2020-04-06
 *
 */
//----------------------------------------------------------------------
#include "pi_hexapod_control/hardware_interface.h"
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

namespace pi_hexapod_control {
HardwareInterface::HardwareInterface()
  : joint_names_(6)
  , visual_joint_names_(36)
  , position_controller_running_(false)
  , referencing_running_(false)
  , prefer_stop_over_halt_(false)
  , control_mode_enabled_(true)
  , latest_error_id_(0)
{
  joint_position_command_.fill(0.0);
  joint_positions_.fill(0.0);
  joint_velocities_.fill(0.0);
  joint_efforts_.fill(0.0);

  visual_joint_positions_.fill(0.0);
  visual_joint_velocities_.fill(0.0);
  visual_joint_efforts_.fill(0.0);
}

bool HardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!visual_joint_generator_.init(root_nh, robot_hw_nh))
  {
    ROS_ERROR_STREAM(
      "Hexapod Visualization will fail: Relevant parameters were not configured properly.");
  }

  /*
   * Get relevant parameter from the parameter server
   */
  bool using_tcp_ip_connection = robot_hw_nh.param("using_tcp_ip_connection", false);
  bool using_rs232_connection  = robot_hw_nh.param("using_rs232_connection", false);
  std::string hexapod_ip;
  int32_t rs232_port_nr, baudrate;

  // check whether or not the connection type is clearly specified
  if (using_tcp_ip_connection == using_rs232_connection)
  {
    ROS_ERROR_COND(
      (!using_tcp_ip_connection && !using_rs232_connection),
      "Either using_tcp_ip_connection or using_rs232_connection parameter needs to be set.");
    ROS_ERROR_COND(
      (using_tcp_ip_connection && using_rs232_connection),
      "Only one of using_tcp_ip_connection and using_rs232_connection parameter can be set.");
    return false;
  }

  // if using TCP/IP check if IP is specified
  if (using_tcp_ip_connection)
  {
    if (!robot_hw_nh.getParam("hexapod_ip", hexapod_ip))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("hexapod_ip")
                                             << " not given.");
      return false;
    }
  }

  // if using RS232 check if port number and baut rate are specified
  if (using_rs232_connection)
  {
    if (!robot_hw_nh.getParam("rs232_port_nr", rs232_port_nr))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("rs232_port_nr")
                                             << " not given.");
      return false;
    }
    if (!robot_hw_nh.getParam("baudrate", baudrate))
    {
      ROS_ERROR_STREAM("Required parameter " << robot_hw_nh.resolveName("baudrate")
                                             << " not given.");
      return false;
    }
  }

  // Get joint names
  if (!root_nh.getParam("hardware_interface/joints", joint_names_))
  {
    ROS_ERROR_STREAM("Cannot find required parameter "
                     << root_nh.resolveName("hardware_interface/joints")
                     << " on the parameter server.");
    return false;
  }

  // get communication_timeout_ms from parameter server
  int32_t communication_timeout_ms = robot_hw_nh.param("communication_timeout_ms", 7000);
  if (communication_timeout_ms <= 0)
  {
    ROS_WARN_STREAM("Invalid communication_timeout_ms of " << communication_timeout_ms
                                                           << " ms, using 7000 ms instead.");
    communication_timeout_ms = 7000;
  }
  ROS_INFO_STREAM("Parameter communication_timeout_ms is set to: " << communication_timeout_ms
                                                                   << " ms.");


  // additional parameter
  uint32_t hexapod_port    = robot_hw_nh.param("hexapod_port", 50000);
  bool is_auto_referencing = robot_hw_nh.param("auto_referencing", false);
  bool is_sim              = robot_hw_nh.param("is_sim", false);
  prefer_stop_over_halt_   = robot_hw_nh.param("prefer_stop_over_halt", false);

  ROS_INFO("Initializing hexapod driver");
  ROS_DEBUG_COND(
    prefer_stop_over_halt_,
    "Prefering STOP over HALT. All calls to haltHexapod() will send STP instead of HLT commands");

  /*
   * Setup Pi Driver
   */

  if (is_sim)
  {
    ROS_INFO("Simulation mode active, no real Hexapod is used.");
    pi_driver_.reset(new SimulatedPiDriver());
  }
  else
  {
    ROS_INFO("Real Hexapod is used, this is no simulation.");
    if (using_tcp_ip_connection)
    {
      pi_driver_.reset(new HardwarePiDriver(communication_timeout_ms, hexapod_ip, hexapod_port));
    }
    else if (using_rs232_connection)
    {
      pi_driver_.reset(new HardwarePiDriver(communication_timeout_ms, rs232_port_nr, baudrate));
    }
    else
    {
      ROS_ERROR("Pi Driver Constructor unclear.");
      return false;
    }
  }
  if (!root_nh.getParam("hardware_interface/visual_joints", visual_joint_names_))
  {
    ROS_ERROR_STREAM("Cannot find required parameter "
                     << root_nh.resolveName("hardware_interface/visual_joints")
                     << " on the parameter server.");
    return false;
  }

  // Setup Driver Error Publishers (latch = true)
  pub_error_id_  = robot_hw_nh.advertise<std_msgs::Int32>("error_id", 10, true);
  pub_error_msg_ = robot_hw_nh.advertise<std_msgs::String>("error_msg", 10, true);

  // Create ros_control interfaces
  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    ROS_DEBUG_STREAM("Registering handles for joint " << joint_names_[i]);

    js_interface_.registerHandle(hardware_interface::JointStateHandle(
      joint_names_[i], &joint_positions_[i], &joint_velocities_[i], &joint_efforts_[i]));

    pj_interface_.registerHandle(hardware_interface::JointHandle(
      js_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]));
  }

  for (std::size_t i = 0; i < visual_joint_names_.size(); ++i)
  {
    ROS_DEBUG_STREAM("Registering only a state handles for joint " << visual_joint_names_[i]);

    js_interface_.registerHandle(hardware_interface::JointStateHandle(visual_joint_names_[i],
                                                                      &visual_joint_positions_[i],
                                                                      &visual_joint_velocities_[i],
                                                                      &visual_joint_efforts_[i]));
  }

  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);

  // Open connection to hexapod
  if (!pi_driver_->connect())
  {
    ROS_ERROR("Unable to connect to Hexapod controller, hardware_interface can't be initialized.");
    return false;
  }

  // Reference hexapod automatically, if required
  if (is_auto_referencing)
  {
    ROS_INFO("Automatic referencing, stand clear of Hexapod hardware!");
    referencing_running_      = true;
    bool reference_successful = pi_driver_->reference();
    referencing_running_      = false;
    if (!reference_successful)
    {
      ROS_ERROR("Referencing failed, Hardware Interface was not initialized.");
      return false;
    }
  }

  // Setup referencing Action Server
  referencing_action_server_.reset(
    new actionlib::SimpleActionServer<pi_hexapod_msgs::ReferencingAction>(
      robot_hw_nh,
      "referencing",
      boost::bind(&HardwareInterface::referencingActionCallback, this, _1),
      false));
  referencing_action_server_->start();

  // Setup enable-control, HALT and STOP Services
  enable_srv_ =
    robot_hw_nh.advertiseService("enable_control", &HardwareInterface::enableServiceCallback, this);
  halt_srv_ =
    robot_hw_nh.advertiseService("halt_hexapod", &HardwareInterface::haltServiceCallback, this);
  stop_srv_ =
    robot_hw_nh.advertiseService("stop_hexapod", &HardwareInterface::stopServiceCallback, this);

  ROS_INFO("Loaded pi_hexapod_control hardware_interface");
  return true;
}

void HardwareInterface::publishDriverStatus()
{
  int current_id = pi_driver_->getErrorID();
  std::string current_msg;
  if (latest_error_id_ == current_id)
  {
    // No need to publish the same error again.
    return;
  }

  current_msg = pi_driver_->getErrorMessage();
  if (current_id == PI_CNTR_NO_ERROR)
  {
    ROS_INFO_STREAM("DriverError(" << latest_error_id_ << ") has been resolved.");
  }
  else
  {
    ROS_ERROR_STREAM("DriverError(" << current_id << "): " << current_msg);
  }
  latest_error_id_ = current_id;

  std_msgs::Int32 error_id_msg;
  std_msgs::String error_msg_msg;
  error_id_msg.data  = current_id;
  error_msg_msg.data = current_msg;
  pub_error_id_.publish(error_id_msg);
  pub_error_msg_.publish(error_msg_msg);
}

void HardwareInterface::referencingActionCallback(
  const pi_hexapod_msgs::ReferencingGoalConstPtr& goal)
{
  pi_hexapod_msgs::ReferencingResult result;

  // Abort if a position controller is running
  if (position_controller_running_)
  {
    ROS_WARN("Cannot reference because of running position controller");
    result.success = false;
    referencing_action_server_->setAborted(result);
    return;
  }

  ROS_INFO("Referencing, stand clear of Hexapod hardware!");
  referencing_running_      = true;
  bool reference_successful = pi_driver_->reference();
  referencing_running_      = false;
  if (!reference_successful)
  {
    ROS_ERROR("Referencing failed, Hardware Interface was not initialized.");
    result.success = false;
    referencing_action_server_->setAborted(result);
    return;
  }
  ROS_INFO("Referencing successfull");
  result.success = true;
  referencing_action_server_->setSucceeded(result);
  publishDriverStatus();
}

bool HardwareInterface::enableServiceCallback(std_srvs::SetBoolRequest& req,
                                              std_srvs::SetBoolResponse& res)
{
  ROS_INFO_STREAM("Control-Mode set to: " << std::boolalpha << (bool)req.data);
  control_mode_enabled_ = (bool)req.data;
  res.success           = true;
  return true;
}

bool HardwareInterface::stopServiceCallback(std_srvs::TriggerRequest& req,
                                            std_srvs::TriggerResponse& res)
{
  control_mode_enabled_ = false;

  if (stopHexapod())
  {
    res.success = true;
  }
  else
  {
    res.success = false;
  }

  publishDriverStatus();
  return true;
}

bool HardwareInterface::haltServiceCallback(std_srvs::TriggerRequest& req,
                                            std_srvs::TriggerResponse& res)
{
  control_mode_enabled_ = false;

  if (haltHexapod())
  {
    res.success = true;
  }
  else
  {
    res.success = false;
  }

  publishDriverStatus();
  return true;
}

void HardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  pi_driver_->requestControllerData(joint_positions_);
  visual_joint_generator_.calculateVisualLinks(joint_positions_, visual_joint_positions_);
}

void HardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  if (control_mode_enabled_)
  {
    pi_driver_->writeControllerCommand(joint_position_command_);
    publishDriverStatus();
  }
}

bool HardwareInterface::prepareSwitch(
  const std::list<hardware_interface::ControllerInfo>& start_list,
  const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  bool ret_val = true;
  if (!start_list.empty())
  {
    for (auto& controller : start_list)
    {
      // check if controller conflicts with referencing
      if (!pi_driver_->isReferenced() || referencing_running_)
      {
        for (auto& resource : controller.claimed_resources)
        {
          if (resource.hardware_interface == "hardware_interface::PositionJointInterface")
          {
            ROS_ERROR_STREAM("Controller " << controller.name
                                           << " cannot be started while the hexapod is unreferenced"
                                           << " or currently referencing.");
            ret_val = false;
          }
        }
      }
    }
  }
  return ret_val;
}

void HardwareInterface::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                 const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  for (auto& controller : stop_list)
  {
    for (auto& resource : controller.claimed_resources)
    {
      if (resource.hardware_interface == "hardware_interface::PositionJointInterface")
      {
        position_controller_running_ = false;
      }
    }
  }

  for (auto& controller : start_list)
  {
    for (auto& resource : controller.claimed_resources)
    {
      if (resource.hardware_interface == "hardware_interface::PositionJointInterface")
      {
        position_controller_running_ = true;
      }
    }
  }
}

bool HardwareInterface::haltHexapod()
{
  return (prefer_stop_over_halt_ ? pi_driver_->stopHexapod() : pi_driver_->haltHexapod());
}

bool HardwareInterface::stopHexapod()
{
  return pi_driver_->stopHexapod();
}

void HardwareInterface::disconnectHexapod()
{
  pi_driver_->disconnect();
}


} // namespace pi_hexapod_control

PLUGINLIB_EXPORT_CLASS(pi_hexapod_control::HardwareInterface, hardware_interface::RobotHW)
