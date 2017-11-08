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

#ifndef IIROB_FILTERS_THRESHOLD_FILTER_H
#define IIROB_FILTERS_THRESHOLD_FILTER_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iirob_filters/ThresholdParameters.h>
#include <filters/filter_base.h>

namespace iirob_filters{
template <typename T>
class ThresholdFilter: public filters::FilterBase<T>
{
public:
        ThresholdFilter();
        ThresholdFilter(ros::NodeHandle nh);
        ThresholdFilter(double threshold);
        ThresholdFilter(double threshold_lin, double threshold_angular);
        
        ~ThresholdFilter();
        virtual bool configure();
        virtual bool update(const T & data_in, T& data_out);
        virtual bool update(const geometry_msgs::WrenchStamped& to_filter_wrench, geometry_msgs::WrenchStamped& filtered_wrench);
    
    private:
        ros::NodeHandle nh_;
        iirob_filters::ThresholdParameters params_;
        double threshold_;
        double threshold_lin_;
        double threshold_angular_;

};

template <typename T>
ThresholdFilter<T>::ThresholdFilter(ros::NodeHandle nh) : nh_(nh), params_{nh_.getNamespace()+"/ThresholdFilter"}
{
/*  params_.fromParamServer();
  threshold_lin_ = params_.linear_threshold;
  threshold_angular_ = params_.angular_threshold;*/
}

template <typename T>
ThresholdFilter<T>::ThresholdFilter(): params_{nh_.getNamespace()+"/ThresholdFilter"}
{
}

template <typename T>
ThresholdFilter<T>::ThresholdFilter(double threshold) : threshold_(threshold), params_{nh_.getNamespace()+"/ThresholdFilter"}
{}
template <typename T>
ThresholdFilter<T>::ThresholdFilter(double threshold_lin, double threshold_angular) : threshold_lin_(threshold_lin), threshold_angular_(threshold_angular), params_{nh_.getNamespace()+"/ThresholdFilter"}
{}

template <typename T>
ThresholdFilter<T>::~ThresholdFilter()
{
}

template <typename T>
bool ThresholdFilter<T>::configure()
{
    params_.fromParamServer();
    threshold_lin_ = params_.linear_threshold;
    threshold_angular_ = params_.angular_threshold;
    
    return true;
}
template <typename T>
bool ThresholdFilter<T>::update(const T & data_in, T& data_out)
{
    data_out = data_in;

  if (fabs(data_in) > threshold_) {
    double sign = (data_in > 0) ? 1 : -1;
    data_out = threshold_*sign;
    
  }
  return true;
}

template <typename T>
bool ThresholdFilter<T>::update(const geometry_msgs::WrenchStamped& to_filter_wrench, geometry_msgs::WrenchStamped& filtered_wrench)
{    
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
  return true;
}


}
#endif
