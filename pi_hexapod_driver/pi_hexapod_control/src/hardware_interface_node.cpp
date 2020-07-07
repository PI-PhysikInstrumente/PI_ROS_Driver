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
 * This file contains the Controller Manager for PI Hexapods.
 *
 * \author  Christian Eichmann <eichmann@fzi.de>
 * \author  Philip Keller <keller@fzi.de>
 * \date    2020-04-06
 *
 */
//----------------------------------------------------------------------

#include <chrono>
#include <controller_manager/controller_manager.h>
#include <csignal>
#include <ros/ros.h>

#include <pi_hexapod_control/hardware_interface.h>

//! global pointer to hardware interface; used in signal handler
std::unique_ptr<pi_hexapod_control::HardwareInterface> g_hw_interface;

/*!
 * \brief Costum signal handler, stopping and disconnecting the hexapod on interupt
 *
 * \param signum signal
 */
void signalHandler(int signum)
{
  // trying to halt Hexapod, if unsuccessful, try sending stop
  if (!g_hw_interface->haltHexapod())
  {
    if (!g_hw_interface->stopHexapod())
    {
      ROS_FATAL("Failed to stop Hexapod, hardware may still be running!");
    }
  }

  g_hw_interface->disconnectHexapod();

  ros::shutdown();
}

/*!
 * \brief main function of this node
 *
 * Sets up ROS, starts async spinners for callback handling,
 creates hardware_interface and acts als controller manager main loop
 *
 * \param argc
 * \param argv
 * \return int
 */
int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "pi_hardware_interface", ros::init_options::NoSigintHandler);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  // get controller_rate from parameter server
  double controller_rate = nh.param("hardware_interface/controller_rate", 100.0);
  if (controller_rate <= 0)
  {
    ROS_WARN_STREAM("Invalid controller_rate of " << controller_rate << ", using 100 instead.");
    controller_rate = 100.0;
  }

  // Setup controller manager and hardware interface
  g_hw_interface.reset(new pi_hexapod_control::HardwareInterface);
  if (!g_hw_interface->init(nh, nh_priv))
  {
    ROS_ERROR_STREAM("Initialization of hexapod failed.");
    exit(1);
  }
  ROS_DEBUG_STREAM("Initialized hardware interface.");
  controller_manager::ControllerManager cm(g_hw_interface.get(), nh);

  ros::Rate rate(controller_rate);
  ros::Time prev_time = ros::Time::now();

  while (!ros::isShuttingDown())
  {
    const ros::Time time       = ros::Time::now();
    const ros::Duration period = time - prev_time;

    g_hw_interface->read(time, period);
    cm.update(time, period);
    g_hw_interface->write(time, period);

    prev_time = time;
    rate.sleep();
  }

  spinner.stop();

  g_hw_interface.reset();

  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");
  return 0;
}
