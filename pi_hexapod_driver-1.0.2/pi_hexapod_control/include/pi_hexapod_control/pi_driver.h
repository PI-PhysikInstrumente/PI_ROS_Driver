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
 * This file contains the abstract PiDriver class and its derived classes for handling both
 * hardware and simulation. It also contains the implementation for the SimulationPiDriver.
 *
 * \author  Christian Eichmann <eichmann@fzi.de>
 * \author  Philip Keller <keller@fzi.de>
 * \date    2020-04-06
 *
 */
//----------------------------------------------------------------------
#ifndef PI_HEXAPOD_CONTROL_PI_DRIVER_H_INCLUDED
#define PI_HEXAPOD_CONTROL_PI_DRIVER_H_INCLUDED

#include "pi_hexapod_control/pi_errors.h"
#include "pi_hexapod_control/types.h"

namespace pi_hexapod_control {

/*!
 * \brief An abstract class representing a Hexapod Driver.
 */
class PiDriver
{
public:
  /*!
   * \brief References the hexapod.
   *
   * \returns True if referenciation successful.
   */
  virtual bool reference() = 0;

  /*!
   * \brief Requests the controller data.
   *
   * \param values Current joint positions
   *
   * \returns True on successful request.
   */
  virtual bool requestControllerData(vector6d_t& values) = 0;

  /*!
   * \brief Writes a joint command together with a keepalive signal onto the socket being sent to
   * the robot.
   *
   * \param values Desired joint positions
   *
   * \returns True on successful write.
   */
  virtual bool writeControllerCommand(const vector6d_t& values) = 0;

  /*!
   * \brief Connects to hexapod.
   *
   * \returns True if connection successful.
   */
  virtual bool connect() = 0;

  /*!
   * \brief Disconnects from hexapod.
   */
  virtual void disconnect() = 0;

  /*!
   * \brief Updates the referenced-status.
   *
   * \return indicates success
   */
  virtual bool updateReferencedStatus() = 0;

  /*!
   * \brief Halts the hexapod.
   *
   * \returns True if request successful.
   */
  virtual bool haltHexapod() = 0;

  /*!
   * \brief Stops the hexapod.
   *
   * \returns True if request successful.
   */
  virtual bool stopHexapod() = 0;

  /*!
   * \brief Provides the current error ID.
   *
   * \returns The PI error ID.
   */
  int getErrorID() { return error_id_; };

  /*!
   * \brief Provides the current error message as string.
   *
   * \returns The translated PI error message.
   */
  std::string getErrorMessage();

  /*!
   * \brief Prints the error to the default log.
   *
   * \returns void
   */
  virtual void updateError() = 0;

  /*!
   * \brief Returns the current referenced-status.
   *
   * \return indicates success
   */
  bool isReferenced() { return is_referenced_; };


protected:
  bool is_referenced_; //!< idicates whether the hexapod is already referenced or not
  int error_id_ = 0;   //!< id of the last received error message
};

/*!
 * \brief This is the main class for interfacing the driver.
 *
 * It sets up all the necessary socket connections and handles the data exchange with the robot.
 * Use this classes methods to access and write data.
 */
class HardwarePiDriver : public PiDriver
{
public:
  /*!
   * \brief Constructs a new PiDriver object.
   *
   * \param communication_timeout_ms timeout in ms
   * \param hexapod_ip IP-address under which the hexapod is reachable.
   * \param hexapod_port Port-address under which the hexapod is reachable.
   */
  HardwarePiDriver(const uint32_t communication_timeout_ms,
                   const std::string& hexapod_ip,
                   const uint32_t hexapod_port = 50000);

  /*!
   * \brief Constructs a new PiDriver object.
   *
   * \param communication_timeout_ms timeout in ms
   * \param rs232_port_nr number of chosen RS232 port.
   * \param baudrate Baudrate for communication with Hexapod Controller.
   */
  HardwarePiDriver(const uint32_t communication_timeout_ms,
                   const int32_t rs232_port_nr,
                   const int32_t baudrate);

  /*!
   * \brief Destructor, disconnects if required.
   *
   * The Destructor disconnects the Driver from the Hexapod controller, if there is a
   * open connection.
   */
  ~HardwarePiDriver();

  /*!
   * \brief References the hexapod.
   *
   * \returns True if referenciation successful.
   */
  bool reference();

  /*!
   * \brief Connects to hexapod.
   *
   * \returns True if connection successful.
   */
  bool connect();

  /*!
   * \brief Disconnects from hexapod.
   */
  void disconnect();

  /*!
   * \brief Requests the controller data.
   *
   * \param values Current joint positions
   *
   * \returns True on successful request.
   */
  bool requestControllerData(vector6d_t& values);

  /*!
   * \brief Writes a joint command together with a keepalive signal onto the socket being sent to
   * the robot.
   *
   * \param values Desired joint positions
   *
   * \returns True on successful write.
   */
  bool writeControllerCommand(const vector6d_t& values);

  /*!
   * \brief Halts the hexapod.
   *
   * \returns True if request successful.
   */
  bool haltHexapod();

  /*!
   * \brief Stops the hexapod.
   *
   * \returns True if request successful.
   */
  bool stopHexapod();

  /*!
   * \brief Updates the referenced-status.
   *
   * \returns True on successful status update call
   */
  bool updateReferencedStatus();

  /*!
   * \brief Prints the error to the default log.
   *
   * \returns void
   */
  void updateError();

private:
  const bool using_tcp_ip_connection_;
  std::string hexapod_ip_;
  uint32_t hexapod_port_;

  const bool using_rs232_connection_;
  const bool error_checking_ = false;
  int32_t rs232_port_nr_;
  int32_t baudrate_;

  bool is_connected_;

  int pi_id_;
  uint32_t communication_timeout_ms_;

  const char axis_[12] = "X Y Z U V W";
  double joint_pos_[6] = {0, 0, 0, 0, 0, 0};
};

/*!
 * \brief Driver with simulated Hexapod hardware.
 *
 * Creates a mock-up driver. No data is read from or writen to hardware.
 * Send commands are stored and used as current position on next cycle.
 * Connections and referencing never fail.
 */
class SimulatedPiDriver : public PiDriver
{
public:
  //! Constructs new Simulated Driver
  SimulatedPiDriver() { mock_values_.fill(0.0); };

  /*!
   * \brief Mock of referencing.
   *
   * \returns True
   */
  bool reference()
  {
    is_referenced_ = true;
    return true;
  };

  /*!
   * \brief Gets dummy controller data.
   *
   * \param values Current joint positions
   *
   * \returns True
   */
  bool requestControllerData(vector6d_t& values)
  {
    values = mock_values_;
    return true;
  };

  /*!
   * \brief Stores a joint command to use as position on next read.
   *
   * \param values Desired joint positions
   *
   * \returns True
   */
  bool writeControllerCommand(const vector6d_t& values)
  {
    mock_values_ = values;
    error_id_    = 0;
    return true;
  };

  /*!
   * \brief Mock of connecting to hexapod.
   *
   * \returns True
   */
  bool connect() { return true; };

  /*!
   * \brief Mock of disconnecting from hexapod.
   */
  void disconnect(){};

  /*!
   * \brief Halts the hexapod.
   *
   * \returns True
   */
  bool haltHexapod()
  {
    error_id_ = 10;
    return true;
  };

  /*!
   * \brief Stops the hexapod.
   *
   * \returns True
   */
  bool stopHexapod()
  {
    error_id_ = 10;
    return true;
  };

  /*!
   * \brief Mock of referenced-status update.
   *
   * \return True
   */
  bool updateReferencedStatus() { return true; };

  /*!
   * \brief Mock of driver error update.
   *
   */
  void updateError() { return; };

private:
  vector6d_t mock_values_;
};

} // namespace pi_hexapod_control
#endif // ifndef PI_HEXAPOD_CONTROL_PI_DRIVER_H_INCLUDED
