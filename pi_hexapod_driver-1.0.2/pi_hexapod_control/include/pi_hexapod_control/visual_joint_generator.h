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
 * This file contains the VisualJointGenerator to calculate joint values for the visual joint of
 * PI Hexapods.
 *
 * \author  Christian Eichmann <eichmann@fzi.de>
 * \author  Philip Keller <keller@fzi.de>
 * \date    2020-04-06
 *
 */
//----------------------------------------------------------------------
#ifndef PI_HEXAPOD_CONTROL_VISUAL_JOINT_GENERATOR_H_INCLUDED
#define PI_HEXAPOD_CONTROL_VISUAL_JOINT_GENERATOR_H_INCLUDED

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "pi_hexapod_control/types.h"
#include <ros/ros.h>

namespace pi_hexapod_control {

/*!
 * \brief An helper class for calculating the anchor joints.
 */
class VisualJointGenerator
{
public:
  //! \brief Constructor
  VisualJointGenerator();

  /*!
   * \brief Initializes the VisualJointGenerator, reads ROS params
   *
   * \param nh Root level ROS node handle
   * \param priv_nh ROS node handle for the node's namespace
   *
   * \returns indicates success
   */
  bool init(ros::NodeHandle& nh, ros::NodeHandle& priv_nh);

  /*!
   * \brief calculates visual link angles for a given 6D platform pose
   *
   * \param current_pos current 6D pose of the platform
   * \param visual_link_pos array in which the calculated angles shall be stored
   * \return indicates success
   */
  bool calculateVisualLinks(const vector6d_t& current_pos, vector36d_t& visual_link_pos);

private:
  /*!
   * \brief Reads parameter of type double, writes log message on error
   *
   * \param priv_nh ROS node handle for the node's namespace
   * \param param_name name of parameter
   * \param value reference to store read parameter
   * \return indicates success
   */
  bool getNamedParamDouble(ros::NodeHandle& priv_nh, const std::string& param_name, double& value);

  // allocate memory for all Eigen objects used for calculating visual links to save time
  std::array<Eigen::Vector3d, 6> base_vector_;
  std::array<Eigen::Vector3d, 6> platform_vector_;

  Eigen::Vector3d base_axis_;
  Eigen::Vector3d base_euler_;
  Eigen::Vector3d current_platform_vector_;
  Eigen::Vector3d platform_axis_;
  Eigen::Vector3d platform_euler_;
  Eigen::Vector3d platform_translation_;
  Eigen::Vector3d z_axis_;

  Eigen::Quaternion<double> platform_rotation_;

  Eigen::Matrix3d rot_matrix_;
};

} // namespace pi_hexapod_control
#endif // ifndef PI_HEXAPOD_CONTROL_VISUAL_JOINT_GENERATOR_H_INCLUDED
