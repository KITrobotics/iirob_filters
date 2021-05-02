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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <filters/filter_base.hpp>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace iirob_filters
{
template <typename T>
class GravityCompensator : public filters::FilterBase<T>
{
public:
  /** \brief Constructor */
  GravityCompensator();

  /** \brief Destructor */
  ~GravityCompensator();

  /** @brief Init node  */
  void setNode(const rclcpp::Node::SharedPtr& node);

  /** @brief Configure filter parameters  */
  virtual bool configure() override;

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const T& data_in, T& data_out) override;

  /** \brief Get most recent parameters */
  void updateParameters();

private:
  /** \brief Dynamic parameter callback activated when parameters change */
  void parameterCallback();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;

  // Storage for Calibration Values
  geometry_msgs::msg::Vector3Stamped cog_;  // Center of Gravity Vector (wrt Sensor Frame)
  double force_z_;                          // Gravitational Force

  // Frames for Transformation of Gravity / CoG Vector
  std::string world_frame_;
  std::string sensor_frame_;

  // tf2 objects
  std::unique_ptr<tf2_ros::Buffer> p_tf_Buffer_;
  std::unique_ptr<tf2_ros::TransformListener> p_tf_Listener_;
  geometry_msgs::msg::TransformStamped transform_, transform_back_;

  // dynamic parameters
  std::unique_ptr<rosparam_shortcuts::NodeParameters> params_;
  bool params_updated_;
  bool initialized_;
  bool configured_;

  uint num_transform_errors_;
};

template <typename T>
GravityCompensator<T>::GravityCompensator()
  : logger_(rclcpp::get_logger("GravityCompensator"))
  , params_updated_(false)
  , initialized_(false)
  , configured_(false)
  , num_transform_errors_(0)
{
}

template <typename T>
GravityCompensator<T>::~GravityCompensator()
{
}

template <typename T>
void GravityCompensator<T>::setNode(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  initialized_ = true;
}

template <typename T>
bool GravityCompensator<T>::configure()
{
  if (!initialized_)
  {
    RCLCPP_INFO(logger_, "Node is not initialized... call setNode()");
    return false;
  }

  p_tf_Buffer_.reset(new tf2_ros::Buffer(node_->get_clock()));
  p_tf_Listener_.reset(new tf2_ros::TransformListener(*p_tf_Buffer_.get(), true));
  params_.reset(new rosparam_shortcuts::NodeParameters(node_, logger_));

  params_->declareAndGet("world_frame", rclcpp::ParameterValue(), rosparam_shortcuts::always_accept);
  params_->declareAndGet("sensor_frame", rclcpp::ParameterValue(), rosparam_shortcuts::always_accept);
  params_->declareAndGet("CoG_x", rclcpp::ParameterValue(), rosparam_shortcuts::always_accept);
  params_->declareAndGet("CoG_y", rclcpp::ParameterValue(), rosparam_shortcuts::always_accept);
  params_->declareAndGet("CoG_z", rclcpp::ParameterValue(), rosparam_shortcuts::always_accept);
  params_->declareAndGet("force", rclcpp::ParameterValue(), rosparam_shortcuts::always_accept);

  params_->registerRosCallbackReconfigure();
  params_->registerUpdateCallback([this]() { parameterCallback(); });

  updateParameters();
  configured_ = true;

  RCLCPP_INFO(logger_,
              "Gravity Compensation Params: world frame: %s; sensor frame: %s; CoG x:%f; "
              "CoG y:%f; CoG z:%f; force: %f.",
              world_frame_.c_str(), sensor_frame_.c_str(), cog_.vector.x, cog_.vector.y, cog_.vector.z, force_z_);

  return true;
}

template <typename T>
bool GravityCompensator<T>::update(const T& data_in, T& data_out)
{
  if (!configured_)
  {
    RCLCPP_ERROR(logger_, "Filter is not configured");
    return false;
  }

  if (params_updated_)
  {
    updateParameters();
  }

  try
  {
    transform_ = p_tf_Buffer_->lookupTransform(world_frame_, data_in.header.frame_id, rclcpp::Time());
    transform_back_ = p_tf_Buffer_->lookupTransform(data_in.header.frame_id, world_frame_, rclcpp::Time());
    num_transform_errors_ = 0;
  }

  catch (const tf2::TransformException& ex)
  {
    if (num_transform_errors_ % 100 == 0)
    {
      RCLCPP_ERROR(logger_, "%s", ex.what());
    }
    num_transform_errors_++;
  }

  geometry_msgs::msg::Vector3Stamped temp_force_transformed, temp_torque_transformed, temp_vector_in, temp_vector_out;

  temp_vector_in.vector = data_in.wrench.force;
  tf2::doTransform(temp_vector_in, temp_force_transformed, transform_);

  temp_vector_in.vector = data_in.wrench.torque;
  tf2::doTransform(temp_vector_in, temp_torque_transformed, transform_);

  // Transform CoG Vector
  geometry_msgs::msg::Vector3Stamped cog_transformed;
  tf2::doTransform(cog_, cog_transformed, transform_);

  // Compensate for gravity force
  temp_force_transformed.vector.z += force_z_;
  // Compensation Values for torque result from Crossprod of cog Vector and (0 0 G)
  temp_torque_transformed.vector.x += (force_z_ * cog_transformed.vector.y);
  temp_torque_transformed.vector.y -= (force_z_ * cog_transformed.vector.x);

  // Copy Message and Compensate values for Gravity Force and Resulting Torque
  // geometry_msgs::WrenchStamped compensated_wrench;
  data_out = data_in;

  tf2::doTransform(temp_force_transformed, temp_vector_out, transform_back_);
  data_out.wrench.force = temp_vector_out.vector;

  tf2::doTransform(temp_torque_transformed, temp_vector_out, transform_back_);
  data_out.wrench.torque = temp_vector_out.vector;

  return true;
}

template <typename T>
void GravityCompensator<T>::updateParameters()
{
  params_updated_ = false;

  world_frame_ = params_->get("world_frame").as_string();
  sensor_frame_ = params_->get("sensor_frame").as_string();
  cog_.vector.x = params_->get("CoG_x").as_double();
  cog_.vector.y = params_->get("CoG_y").as_double();
  cog_.vector.z = params_->get("CoG_z").as_double();
  force_z_ = params_->get("force").as_double();
}

template <typename T>
void GravityCompensator<T>::parameterCallback()
{
  params_updated_ = true;
}

}  // namespace iirob_filters
#endif
