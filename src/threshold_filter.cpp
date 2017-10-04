/*****************************************************************************
 *
 * Copyright 2016 Intelligent Industrial Robotics (IIROB) Group,
 * Institute for Anthropomatics and Robotics (IAR) -
 * Intelligent Process Control and Robotics (IPR),
 * Karlsruhe Institute of Technology (KIT)
 *
 * Author: Alexandar Pollmann
 *         Denis Štogl, email: denis.stogl@kit.edu
 *
 * Date of creation: 2015-2016
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
//#include <pluginlib/class_list_macros.h>
#include <iirob_filters/threshold_filter.h>
namespace iirob_filters{
ThresholdFilter::ThresholdFilter(ros::NodeHandle nh) : nh_(nh), params_{nh_.getNamespace()+"/ThresholdFilter"}
{
  params_.fromParamServer();
  threshold_lin_ = params_.linear_threshold;
  threshold_angular_ = params_.angular_threshold;
}
ThresholdFilter::ThresholdFilter(): params_{nh_.getNamespace()+"/ThresholdFilter"}
{}

bool ThresholdFilter::init(const ros::NodeHandle &nh)
{
  params_.fromParamServer();
  threshold_lin_ = params_.linear_threshold;
  threshold_angular_ = params_.angular_threshold;
}

ThresholdFilter::ThresholdFilter(double threshold) : threshold_(threshold), params_{nh_.getNamespace()+"/ThresholdFilter"}
{}

ThresholdFilter::ThresholdFilter(double threshold_lin, double threshold_angular) : threshold_lin_(threshold_lin), threshold_angular_(threshold_angular), params_{nh_.getNamespace()+"/ThresholdFilter"}
{}

double ThresholdFilter::applyFilter(double value)
{
  double filtered_value = value;

  if (fabs(value) > threshold_) {
    double sign = (value > 0) ? 1 : -1;
    filtered_value = threshold_*sign;
  }

  return filtered_value;
}

geometry_msgs::WrenchStamped ThresholdFilter::applyFilter(const geometry_msgs::WrenchStamped& to_filter_wrench)
{
    geometry_msgs::WrenchStamped filtered_wrench;
    filtered_wrench.header=to_filter_wrench.header;

    if (fabs(to_filter_wrench.wrench.force.x) > threshold_lin_)
    {
        double sign = (to_filter_wrench.wrench.force.x > 0) ? 1 : -1;
        filtered_wrench.wrench.force.x = to_filter_wrench.wrench.force.x-threshold_lin_*sign;
    }
    if (fabs(to_filter_wrench.wrench.force.y) > threshold_lin_)
    {
        double sign = (to_filter_wrench.wrench.force.y > 0) ? 1 : -1;
        filtered_wrench.wrench.force.y = to_filter_wrench.wrench.force.y-threshold_lin_*sign;
    }
    if (fabs(to_filter_wrench.wrench.force.z) > threshold_lin_)
    {
        double sign = (to_filter_wrench.wrench.force.z > 0) ? 1 : -1;
        filtered_wrench.wrench.force.z = to_filter_wrench.wrench.force.z-threshold_lin_*sign;
    }
    if (fabs(to_filter_wrench.wrench.torque.x) > threshold_angular_)
    {
        double sign = (to_filter_wrench.wrench.torque.x > 0) ? 1 : -1;
        filtered_wrench.wrench.torque.x = to_filter_wrench.wrench.torque.x-threshold_angular_*sign;
    }
    if (fabs(to_filter_wrench.wrench.torque.y) > threshold_angular_)
    {
        double sign = (to_filter_wrench.wrench.force.y > 0) ? 1 : -1;
        filtered_wrench.wrench.torque.y = to_filter_wrench.wrench.torque.y-threshold_angular_*sign;
    }
    if (fabs(to_filter_wrench.wrench.torque.z) > threshold_angular_)
    {
        double sign = (to_filter_wrench.wrench.torque.z > 0) ? 1 : -1;
        filtered_wrench.wrench.torque.z = to_filter_wrench.wrench.torque.z-threshold_angular_*sign;
    }

    return filtered_wrench;
}
}
//PLUGINLIB_EXPORT_CLASS(ThresholdFilter, interface)
