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
 * This file contains the implementation of VisualJointGenerator.
 *
 * \author  Christian Eichmann <eichmann@fzi.de>
 * \author  Philip Keller <keller@fzi.de>
 * \date    2020-04-29
 *
 */
//----------------------------------------------------------------------

#include "pi_hexapod_control/visual_joint_generator.h"
#include <math.h>

namespace pi_hexapod_control {

VisualJointGenerator::VisualJointGenerator()
  : z_axis_(0.0, 0.0, 1.0){};

bool VisualJointGenerator::getNamedParamDouble(ros::NodeHandle& priv_nh,
                                               const std::string& param_name,
                                               double& value)
{
  if (priv_nh.getParam(param_name, value))
  {
    return true;
  }

  ROS_ERROR_STREAM("Cannot find required parameter " << priv_nh.resolveName(param_name)
                                                     << " on the parameter server.");
  return false;
}

bool VisualJointGenerator::init(ros::NodeHandle& nh, ros::NodeHandle& priv_nh)
{
  const int x{0}, y{1}, z{2};

  bool ret_val = true;

  // for every axis, create base and platform vector from parameter
  for (size_t axis_id = 0; axis_id < 6; axis_id++)
  {
    const std::string axis_string = "axis_" + std::to_string(axis_id);
    ret_val =
      ret_val &&
      getNamedParamDouble(priv_nh, axis_string + "/base_anchor/x", base_vector_[axis_id][x]) &&
      getNamedParamDouble(priv_nh, axis_string + "/base_anchor/y", base_vector_[axis_id][y]) &&
      getNamedParamDouble(priv_nh, axis_string + "/base_anchor/z", base_vector_[axis_id][z]) &&
      getNamedParamDouble(
        priv_nh, axis_string + "/platform_anchor/x", platform_vector_[axis_id][x]) &&
      getNamedParamDouble(
        priv_nh, axis_string + "/platform_anchor/y", platform_vector_[axis_id][y]) &&
      getNamedParamDouble(
        priv_nh, axis_string + "/platform_anchor/z", platform_vector_[axis_id][z]);
  }

  // return true, if all parameters were read successful
  return ret_val;
};


bool VisualJointGenerator::calculateVisualLinks(const vector6d_t& current_pos,
                                                vector36d_t& visual_link_pos)
{
  // current platform position as translation vector
  platform_translation_ << current_pos[0], current_pos[1], current_pos[2];

  // current platform rotation from RPY
  platform_rotation_ = Eigen::AngleAxisd(current_pos[3], Eigen::Vector3d::UnitX()) *
                       Eigen::AngleAxisd(current_pos[4], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(current_pos[5], Eigen::Vector3d::UnitZ());

  for (size_t axis_id = 0; axis_id < 6; axis_id++)
  {
    // calculate platform anchor position in relation to zero_frame
    // dependent on current translation and rotation
    current_platform_vector_ =
      platform_translation_ + platform_rotation_ * platform_vector_[axis_id];

    // calculate vector from base anchor point to current platform anchor point
    base_axis_ = current_platform_vector_ - base_vector_[axis_id];
    // calculate euler angles between base_axis_ and z_axis_ (0, 0, 1)
    base_euler_ = Eigen::Quaternion<double>::FromTwoVectors(z_axis_, base_axis_)
                    .toRotationMatrix()
                    .eulerAngles(0, 1, 2);
    // Use calculated angles as joint states for visual links
    visual_link_pos[6 * axis_id + 0] = base_euler_[0];
    visual_link_pos[6 * axis_id + 1] = base_euler_[1];
    visual_link_pos[6 * axis_id + 2] = base_euler_[2];

    // calculate vector from current platform anchor point to base anchor point
    platform_axis_ = base_vector_[axis_id] - current_platform_vector_;
    // calculate euler angles between z_axis_ (0, 0, 1) and platform_axis_
    rot_matrix_ =
      Eigen::Quaternion<double>::FromTwoVectors(z_axis_, platform_axis_).toRotationMatrix();
    platform_euler_ = (platform_rotation_.inverse() * rot_matrix_).eulerAngles(0, 1, 2);
    // Use calculated angles as joint states for visual links
    visual_link_pos[6 * axis_id + 3] = platform_euler_[0];
    visual_link_pos[6 * axis_id + 4] = platform_euler_[1];
    visual_link_pos[6 * axis_id + 5] = 0.0; // axis rotation in Z is fixed
  }
  return true;
};


} // namespace pi_hexapod_control
