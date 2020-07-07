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
 * This file contains the Implementation of HardwarePiDriver.
 *
 * \author  Christian Eichmann <eichmann@fzi.de>
 * \author  Philip Keller <keller@fzi.de>
 * \date    2020-04-06
 *
 */
//----------------------------------------------------------------------

#include "pi_hexapod_control/pi_driver.h"
#include "pi_hexapod_control/log.h"
#include "pi_hexapod_control/pi_errors.h"
#include <PI_GCS2_DLL.h>
#include <algorithm>

namespace pi_hexapod_control {

std::string PiDriver::getErrorMessage()
{
  auto error_obj = g_PI_ERRORS_UMAP.find(error_id_);
  if (error_obj != g_PI_ERRORS_UMAP.end())
  {
    // handle common error with elaborate log message
    if (error_id_ == PI_UNEXPECTED_RESPONSE)
    {
      LOG_ERROR_STREAM("# PI-Hexapod-Error ("
                       << error_id_ << "): Maybe the Communication Timeout is too small.");
    }

    // return error message string
    return error_obj->second;
  }

  LOG_ERROR_STREAM("Unknown Error with ID: " << error_id_);
  return "Unknown error";
}

HardwarePiDriver::HardwarePiDriver(const uint32_t communication_timeout_ms,
                                   const std::string& hexapod_ip,
                                   const uint32_t hexapod_port)
  : using_tcp_ip_connection_(true)
  , using_rs232_connection_(false)
  , hexapod_ip_(hexapod_ip)
  , hexapod_port_(hexapod_port)
  , is_connected_(false)
  , communication_timeout_ms_(communication_timeout_ms)
{
  LOG_DEBUG("Initializing TCP-PiDriver.");
  LOG_DEBUG("Initialization done.");
}

HardwarePiDriver::HardwarePiDriver(const uint32_t communication_timeout_ms,
                                   const int32_t rs232_port_nr,
                                   const int32_t baudrate)
  : using_tcp_ip_connection_(false)
  , using_rs232_connection_(true)
  , rs232_port_nr_(rs232_port_nr)
  , baudrate_(baudrate)
  , is_connected_(false)
  , communication_timeout_ms_(communication_timeout_ms)
{
  LOG_DEBUG("Initializing Serial-PiDriver.");
  LOG_DEBUG("Initialization done.");
}

HardwarePiDriver::~HardwarePiDriver()
{
  LOG_DEBUG("Destroying PiDriver.");
  if (is_connected_)
  {
    disconnect();
  }
}

bool HardwarePiDriver::connect()
{
  // setup connection
  if (using_tcp_ip_connection_)
  {
    pi_id_ = PI_ConnectTCPIP(const_cast<char*>(hexapod_ip_.c_str()), hexapod_port_);
  }
  else if (using_rs232_connection_)
  {
    pi_id_ = PI_ConnectRS232(rs232_port_nr_, baudrate_);
  }
  else
  {
    LOG_ERROR("Communication protocol not specified.");
    return false;
  }

  // pi_id_ below 0 indicates failure
  if (pi_id_ < 0)
  {
    LOG_ERROR_STREAM("Invalid Hexapod ID: " << pi_id_);
    LOG_ERROR_STREAM_COND(using_tcp_ip_connection_,
                          "Connecting with the Hexapod failed with TCP/IP "
                            << hexapod_ip_.c_str() << ":" << hexapod_port_);
    LOG_ERROR_STREAM_COND(using_rs232_connection_,
                          "Connecting with the Hexapod failed with RS232 Port "
                            << rs232_port_nr_ << " and baudrate " << baudrate_);
    updateError();
    return false;
  }

  // configure timeout and error checking behavior
  PI_SetTimeout(pi_id_, communication_timeout_ms_);
  PI_EnableReconnect(pi_id_, FALSE);
  PI_SetNrTimeoutsBeforeClose(pi_id_, 1);
  BOOL prev_error_checking = PI_SetErrorCheck(pi_id_, error_checking_ ? TRUE : FALSE);
  LOG_INFO_STREAM("Hexapod-ErrorCheck is set to " << std::boolalpha << error_checking_
                                                  << ", it was: " << (prev_error_checking == TRUE));

  is_connected_ = true;
  return true;
}

bool HardwarePiDriver::requestControllerData(vector6d_t& values)
{
  // request current position
  if (PI_qPOS(pi_id_, axis_, joint_pos_) == FALSE)
  {
    LOG_ERROR("Requesting Data from the Hexapod failed.");
    updateError();
    return false;
  }

  // transform values from millimeter to meter
  values[0] = joint_pos_[0] / 1000;
  values[1] = joint_pos_[1] / 1000;
  values[2] = joint_pos_[2] / 1000;
  // transform values from degress to radians
  values[3] = joint_pos_[3] * M_PI / 180;
  values[4] = joint_pos_[4] * M_PI / 180;
  values[5] = joint_pos_[5] * M_PI / 180;
  return true;
}

bool HardwarePiDriver::writeControllerCommand(const vector6d_t& values)
{
  if (!isReferenced())
  {
    LOG_WARN_ONCE("Hexapod is not referenced. Writing commands not available in this state.");
    return false;
  }

  vector6d_t values_in_mm_and_degree;
  // transform values from meter to millimeter
  values_in_mm_and_degree[0] = values[0] * 1000;
  values_in_mm_and_degree[1] = values[1] * 1000;
  values_in_mm_and_degree[2] = values[2] * 1000;
  // transform values from radians to degrees
  values_in_mm_and_degree[3] = values[3] / M_PI * 180;
  values_in_mm_and_degree[4] = values[4] / M_PI * 180;
  values_in_mm_and_degree[5] = values[5] / M_PI * 180;

  // command move to absolute position
  if (PI_MOV(pi_id_, axis_, values_in_mm_and_degree.data()) == FALSE)
  {
    LOG_ERROR("Writing Command to the Hexapod failed.");
    updateError();
    return false;
  }

  if (!error_checking_)
  {
    updateError();
  }

  return true;
}

void HardwarePiDriver::disconnect()
{
  PI_CloseConnection(pi_id_);
  LOG_INFO("Disconnected from Hexapod Controller.");
  is_connected_ = false;
}

bool HardwarePiDriver::updateReferencedStatus()
{
  int referenced;
  char x_axis[2] = "X";
  if (!is_connected_)
  {
    LOG_ERROR("Hexapod-Reference request failed, because there is no connection.");
    updateError();
    return false;
  }

  // query referencing status
  if (PI_qFRF(pi_id_, axis_, &referenced) == FALSE)
  {
    LOG_ERROR("Hexapod-Reference request failed.");
    updateError();
    return false;
  }

  is_referenced_ = (referenced == TRUE);
  LOG_INFO_STREAM("Hexapod-Reference request successful. Status: " << std::boolalpha
                                                                   << is_referenced_);

  return true;
}

bool HardwarePiDriver::reference()
{
  updateReferencedStatus();

  if (isReferenced())
  {
    LOG_INFO("Hexapod is already referenced.");
    return true;
  }

  // reference the axis using the refence switch
  int flag;
  char x_axis[2] = "X";
  LOG_INFO("Referencing hexapod.");
  if (PI_FRF(pi_id_, x_axis) == FALSE)
  {
    LOG_ERROR("Hexapod Axis-Referencing failed.");
    updateError();
    return false;
  }

  // Wait until the reference move is done.
  flag = 0;
  while (flag == 0)
  {
    if (PI_IsControllerReady(pi_id_, &flag) == FALSE)
    {
      LOG_ERROR("Hexapod Axis-Referencing failed to finish.");
      updateError();
      return false;
    }
  }

  updateReferencedStatus();

  LOG_INFO("Hexapod Axis Referencing completed.");
  return true;
}

bool HardwarePiDriver::haltHexapod()
{
  BOOL response = PI_HLT(pi_id_, axis_);
  if (response == TRUE)
  {
    LOG_INFO("Halted the Hexapod.");
    updateError();
  }
  else
  {
    LOG_ERROR("Failed to halt the Hexapod.");
    updateError();
  }
  return (response == TRUE);
}

bool HardwarePiDriver::stopHexapod()
{
  BOOL response = PI_STP(pi_id_);
  if (response == TRUE)
  {
    LOG_INFO("Stopped the Hexapod.");
    updateError();
  }
  else
  {
    LOG_ERROR("Failed to stop the Hexapod.");
    updateError();
  }
  return (response == TRUE);
}

void HardwarePiDriver::updateError()
{
  error_id_ = PI_GetError(pi_id_);
}

} // namespace pi_hexapod_control
