/*****************************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology (KIT)
 *
 * Author: Denis Štogl, email: denis.stogl@kit.edu
 *
 * Date of creation: 2016
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

#ifndef IIROB_FILTERS_LOW_PASS_FILTER_H
#define IIROB_FILTERS_LOW_PASS_FILTER_H

#include <cmath>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <filters/filter_base.hpp>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace iirob_filters
{
template <typename T>
class LowPassFilter : public filters::FilterBase<T>
{
public:
  LowPassFilter();

  ~LowPassFilter();

  /** @brief Init node  */
  void setNode(const rclcpp::Node::SharedPtr& node);

  virtual bool configure();

  bool configure(const std::string& ns);

  virtual bool update(const T& data_in, T& data_out);

  /** \brief Get most recent parameters */
  void updateParameters();

private:
  /** \brief Dynamic parameter callback activated when parameters change */
  void parameterCallback();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;

  // Parameters
  std::string parameter_namespace_;
  double sampling_frequency_;
  double damping_frequency_;
  double damping_intensity_;
  int divider_;

  // Filter parameters
  double b1;
  double a1;
  double filtered_value, filtered_old_value, old_value;
  Eigen::Matrix<double, 6, 1> msg_filtered, msg_filtered_old, msg_old;

  // dynamic parameters
  std::unique_ptr<rosparam_shortcuts::NodeParameters> params_;
  bool params_updated_;
  bool initialized_;
  bool configured_;
};

template <typename T>
LowPassFilter<T>::LowPassFilter()
  : logger_(rclcpp::get_logger("LowPassFilter")), params_updated_(false), initialized_(false), configured_(false)
{
}

template <typename T>
LowPassFilter<T>::~LowPassFilter()
{
}

template <typename T>
void LowPassFilter<T>::setNode(const rclcpp::Node::SharedPtr& node)
{
  node_ = node;
  initialized_ = true;
}

template <typename T>
bool LowPassFilter<T>::configure()
{
  configure("");
  return true;
}

template <typename T>
bool LowPassFilter<T>::configure(const std::string& ns)
{
  if (!initialized_)
  {
    RCLCPP_INFO(logger_, "Node is not initialized... call setNode()");
    return false;
  }

  params_.reset(new rosparam_shortcuts::NodeParameters(node_, logger_));

  parameter_namespace_ = ns;
  params_->declareAndGet(parameter_namespace_ + "sampling_frequency", rclcpp::ParameterValue(),
                         rosparam_shortcuts::always_accept);
  params_->declareAndGet(parameter_namespace_ + "damping_frequency", rclcpp::ParameterValue(),
                         rosparam_shortcuts::always_accept);
  params_->declareAndGet(parameter_namespace_ + "damping_intensity", rclcpp::ParameterValue(),
                         rosparam_shortcuts::always_accept);
  params_->declareAndGet(parameter_namespace_ + "divider", rclcpp::ParameterValue(), rosparam_shortcuts::always_accept);

  updateParameters();
  configured_ = true;

  RCLCPP_INFO(logger_,
              "Low Pass Filter Params: sampling frequency: %f; damping frequency: %f; damping intensity :%f; "
              "divider :%d",
              sampling_frequency_, damping_frequency_, damping_intensity_, divider_);

  // Initialize storage Vectors
  filtered_value = filtered_old_value = old_value = 0;
  for (unsigned int i = 0; i < 6; i++)
  {
    msg_filtered(i) = msg_filtered_old(i) = msg_old(i) = 0;
  }

  params_->registerRosCallbackReconfigure();
  params_->registerUpdateCallback([this]() { parameterCallback(); });

  return true;
}

template <>
inline bool LowPassFilter<geometry_msgs::msg::WrenchStamped>::update(const geometry_msgs::msg::WrenchStamped& data_in,
                                                                     geometry_msgs::msg::WrenchStamped& data_out)
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

  // IIR Filter
  msg_filtered = b1 * msg_old + a1 * msg_filtered_old;
  msg_filtered_old = msg_filtered;

  // TODO use wrenchMsgToEigen
  msg_old[0] = data_in.wrench.force.x;
  msg_old[1] = data_in.wrench.force.y;
  msg_old[2] = data_in.wrench.force.z;
  msg_old[3] = data_in.wrench.torque.x;
  msg_old[4] = data_in.wrench.torque.y;
  msg_old[5] = data_in.wrench.torque.z;

  data_out.wrench.force.x = msg_filtered[0];
  data_out.wrench.force.y = msg_filtered[1];
  data_out.wrench.force.z = msg_filtered[2];
  data_out.wrench.torque.x = msg_filtered[3];
  data_out.wrench.torque.y = msg_filtered[4];
  data_out.wrench.torque.z = msg_filtered[5];
  return true;
}

template <typename T>
bool LowPassFilter<T>::update(const T& data_in, T& data_out)
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

  data_out = b1 * old_value + a1 * filtered_old_value;
  filtered_old_value = data_out;
  old_value = data_in;

  return true;
}

template <typename T>
void LowPassFilter<T>::updateParameters()
{
  params_updated_ = false;

  sampling_frequency_ = params_->get(parameter_namespace_ + "sampling_frequency").as_double();
  damping_frequency_ = params_->get(parameter_namespace_ + "damping_frequency").as_double();
  damping_intensity_ = params_->get(parameter_namespace_ + "damping_intensity").as_double();
  divider_ = params_->get(parameter_namespace_ + "divider").as_int();

  a1 = exp(-1.0 / sampling_frequency_ * (2.0 * M_PI * damping_frequency_) / (pow(10.0, damping_intensity_ / -10.0)));
  b1 = 1.0 - a1;
}

template <typename T>
void LowPassFilter<T>::parameterCallback()
{
  params_updated_ = true;
}

}  // namespace iirob_filters
#endif
