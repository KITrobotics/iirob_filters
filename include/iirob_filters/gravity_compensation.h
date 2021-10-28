/*****************************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology (KIT)
 *
 * Author: Andreea Tulbure, email: andreea.tulbure@student.kit.edu
 *         Denis Štogl, email: denis.stogl@kit.edu
 *         Alexandar Pollmann
 *
 * Date of creation: 2015-2017
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided
 * that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This package is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifndef IIROB_FILTERS_GRAVITY_COMPENSATION_H
#define IIROB_FILTERS_GRAVITY_COMPENSATION_H

#include <boost/scoped_ptr.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <filters/filter_base.hpp>

#include "controller_interface/controller_parameters.hpp"

namespace iirob_filters
{
class GravityCompensatorParameters : public controller_interface::ControllerParameters
{
public:
  GravityCompensatorParameters(const std::string & params_prefix) :
   controller_interface::ControllerParameters(params_prefix, 0, 4, 3)
  {
    add_string_parameter("world_frame", false);
    add_string_parameter("sensor_frame", false);
    add_string_parameter("force_frame", false);

    add_double_parameter("CoG_x", true);
    add_double_parameter("CoG_y", true);
    add_double_parameter("CoG_z", true);
    add_double_parameter("force", true);
  }

  bool check_if_parameters_are_valid() override
  {
    bool ret = true;

    // Check if any string parameter is empty
    ret = !empty_parameter_in_list(string_parameters_);

    for (size_t i = 0; i < 4; ++i)
    {
      if (std::isnan(double_parameters_[i].second))
      {
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(),
          "Parameter '%s' has to be set", double_parameters_[i].first.name.c_str());
        ret = false;
      }
    }

    return ret;
  }

  void update_storage() override
  {
    world_frame_ = string_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "World frame: %s", world_frame_.c_str());
    sensor_frame_ = string_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "Sensor frame: %s", sensor_frame_.c_str());
    force_frame_ = string_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "Force frame: %s", force_frame_.c_str());

    cog_.vector.x = double_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "CoG X is %e", cog_.vector.x);
    cog_.vector.y = double_parameters_[1].second;
    RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "CoG Y is %e", cog_.vector.y);
    cog_.vector.z = double_parameters_[2].second;
    RCUTILS_LOG_INFO_NAMED(
        logger_name_.c_str(),
       "CoG Z is %e", cog_.vector.z);

    force_z_ = double_parameters_[3].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Force is %e", force_z_);
  }

  // Frames for Transformation of Gravity / CoG Vector
  std::string world_frame_;
  std::string sensor_frame_;
  std::string force_frame_;

  // Storage for Calibration Values
  geometry_msgs::msg::Vector3Stamped cog_;  // Center of Gravity Vector (wrt Sensor Frame)
  double force_z_;                          // Gravitational Force
};

template <typename T>
class GravityCompensator : public filters::FilterBase<T>
{
public:
  /** \brief Constructor */
  GravityCompensator();

  /** \brief Destructor */
  ~GravityCompensator();

  /** @brief Configure filter parameters  */
  virtual bool configure() override;

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const T& data_in, T& data_out) override;

  /** \brief Get most recent parameters */
  bool updateParameters();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
//   // Storage for Calibration Values
//   geometry_msgs::msg::Vector3Stamped cog_;  // Center of Gravity Vector (wrt Sensor Frame)
//   double force_z_;                          // Gravitational Force
//
//   // Frames for Transformation of Gravity / CoG Vector
//   std::string world_frame_;
//   std::string sensor_frame_;
//   std::string force_frame_;

  std::unique_ptr<GravityCompensatorParameters> parameters_;

  // tf2 objects
  std::unique_ptr<tf2_ros::Buffer> p_tf_Buffer_;
  std::unique_ptr<tf2_ros::TransformListener> p_tf_Listener_;
  geometry_msgs::msg::TransformStamped transform_, transform_back_, transform_cog_;

  // Callback for updating dynamic parameters
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_callback_handle_;
};

template <typename T>
GravityCompensator<T>::GravityCompensator()
  : logger_(rclcpp::get_logger("GravityCompensator"))
{
}

template <typename T>
GravityCompensator<T>::~GravityCompensator()
{
}

template <typename T>
bool GravityCompensator<T>::configure()
{
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  p_tf_Buffer_.reset(new tf2_ros::Buffer(clock_));
  p_tf_Listener_.reset(new tf2_ros::TransformListener(*p_tf_Buffer_.get(), true));

  parameters_.reset(new GravityCompensatorParameters(this->param_prefix_));

  parameters_->declare_parameters(this->logging_interface_, this->params_interface_);

  if(!parameters_->get_parameters())
  {
    return false;
  }

  // Add callback to dynamically update parameters
  on_set_callback_handle_ = this->params_interface_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {

      return parameters_->set_parameter_callback(parameters);
    });

//   RCLCPP_INFO(logger_,
//               "Gravity Compensation Params: world frame: %s; sensor frame: %s; force frame: %s; CoG x:%f; "
//               "CoG y:%f; CoG z:%f; force: %f.",
//               world_frame_.c_str(), sensor_frame_.c_str(), force_frame_.c_str(), cog_.vector.x, cog_.vector.y, cog_.vector.z, force_z_);

  return true;
}

template <typename T>
bool GravityCompensator<T>::update(const T& data_in, T& data_out)
{
  if (!this->configured_)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE(logger_, *clock_, 2000, "Filter is not configured");
    return false;
  }

  parameters_->update();

  try
  {
    transform_ = p_tf_Buffer_->lookupTransform(
      parameters_->world_frame_, data_in.header.frame_id, rclcpp::Time());
    transform_back_ = p_tf_Buffer_->lookupTransform(
      data_in.header.frame_id, parameters_->world_frame_, rclcpp::Time());
    transform_cog_ = p_tf_Buffer_->lookupTransform(
      parameters_->world_frame_,  parameters_->force_frame_, rclcpp::Time());
  }
  catch (const tf2::TransformException& ex)
  {
      RCLCPP_ERROR_SKIPFIRST_THROTTLE(logger_, *clock_, 5000, "%s", ex.what());
  }

  geometry_msgs::msg::Vector3Stamped temp_force_transformed, temp_torque_transformed, temp_vector_in, temp_vector_out;

  temp_vector_in.vector = data_in.wrench.force;
  tf2::doTransform(temp_vector_in, temp_force_transformed, transform_);

  temp_vector_in.vector = data_in.wrench.torque;
  tf2::doTransform(temp_vector_in, temp_torque_transformed, transform_);

  // Transform CoG Vector
  geometry_msgs::msg::Vector3Stamped cog_transformed;
  tf2::doTransform(parameters_->cog_, cog_transformed, transform_cog_);

  // Compensate for gravity force
  temp_force_transformed.vector.z += parameters_->force_z_;
  // Compensation Values for torque result from Crossprod of cog Vector and (0 0 G)
  temp_torque_transformed.vector.x += (parameters_->force_z_ * cog_transformed.vector.y);
  temp_torque_transformed.vector.y -= (parameters_->force_z_ * cog_transformed.vector.x);

  // Copy Message and Compensate values for Gravity Force and Resulting Torque
  data_out = data_in;

  tf2::doTransform(temp_force_transformed, temp_vector_out, transform_back_);
  data_out.wrench.force = temp_vector_out.vector;

  tf2::doTransform(temp_torque_transformed, temp_vector_out, transform_back_);
  data_out.wrench.torque = temp_vector_out.vector;

  return true;
}

// template <typename T>
// bool GravityCompensator<T>::updateParameters()
// {
//   if (!filters::FilterBase<T>::getParam("world_frame", world_frame_)) {
//     RCLCPP_ERROR(
//       this->logging_interface_->get_logger(),
//       "Gravitiy Compensator did not find param world_frame_");
//     return false;
//   }
//   if (!filters::FilterBase<T>::getParam("sensor_frame", sensor_frame_)) {
//     RCLCPP_ERROR(
//       this->logging_interface_->get_logger(),
//       "Gravitiy Compensator did not find param sensor_frame");
//     return false;
//   }
//   if (!filters::FilterBase<T>::getParam("force_frame", force_frame_)) {
//     RCLCPP_ERROR(
//       this->logging_interface_->get_logger(),
//       "Gravitiy Compensator did not find param force_frame");
//     return false;
//   }
//   if (!filters::FilterBase<T>::getParam("CoG_x", cog_.vector.x)) {
//     RCLCPP_ERROR(
//       this->logging_interface_->get_logger(),
//       "Gravitiy Compensator did not find param CoG_x");
//     return false;
//   }
//   if (!filters::FilterBase<T>::getParam("CoG_y", cog_.vector.y)) {
//     RCLCPP_ERROR(
//       this->logging_interface_->get_logger(),
//       "Gravitiy Compensator did not find param CoG_y");
//     return false;
//   }
//   if (!filters::FilterBase<T>::getParam("CoG_z", cog_.vector.z)) {
//     RCLCPP_ERROR(
//       this->logging_interface_->get_logger(),
//       "Gravitiy Compensator did not find param CoG_z");
//     return false;
//   }
//   if (!filters::FilterBase<T>::getParam("force", force_z_)) {
//     RCLCPP_ERROR(
//       this->logging_interface_->get_logger(),
//       "Gravitiy Compensator did not find param force");
//     return false;
//   }
//   return true;
// }
}  // namespace iirob_filters
#endif
