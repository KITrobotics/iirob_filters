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
#include <iirob_filters/ThresholdConfig.h>
#include <dynamic_reconfigure/server.h>
#include <filters/filter_base.h>

namespace iirob_filters{
template <typename T>
class ThresholdFilter: public filters::FilterBase<T>
{
public:
        ThresholdFilter();
        
        ~ThresholdFilter();
        virtual bool configure();
        virtual bool update(const T & data_in, T& data_out);
    
    private:
        iirob_filters::ThresholdParameters params_;
        double threshold_;
        double threshold_lin_;
        double threshold_angular_;
                  
        dynamic_reconfigure::Server<iirob_filters::ThresholdConfig> reconfigCalibrationSrv_; // Dynamic reconfiguration service        

        void reconfigureConfigurationRequest(iirob_filters::ThresholdConfig& config, uint32_t level);

};

template <typename T>
ThresholdFilter<T>::ThresholdFilter(): params_{ros::NodeHandle("~/ThresholdFilter/params").getNamespace()}
{
    reconfigCalibrationSrv_.setCallback(boost::bind(&ThresholdFilter<T>::reconfigureConfigurationRequest, this, _1, _2));
}

template <typename T>
ThresholdFilter<T>::~ThresholdFilter()
{
}

template <typename T>
bool ThresholdFilter<T>::configure()
{
    params_.fromParamServer();
    threshold_ = params_.threshold;
    threshold_lin_ = params_.linear_threshold;
    threshold_angular_ = params_.angular_threshold;
    if(threshold_lin_ == 0)
	  ROS_ERROR("ThresholdFilter did not find param linear_threshold");
    if(threshold_angular_ == 0)
	  ROS_ERROR("ThresholdFilter did not find param angular_threshold");
    
    ROS_INFO("Threshhold Filter Params: Threshold: %f; Treshold lin.: %f; Threshold Anglular: %f" ,
    threshold_, threshold_lin_, threshold_angular_);
    
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

template <>
inline bool ThresholdFilter<geometry_msgs::WrenchStamped>::update(const geometry_msgs::WrenchStamped & data_in, geometry_msgs::WrenchStamped& data_out)
{    
    data_out = data_in;

    if (fabs(data_in.wrench.force.x) > threshold_lin_)
    {
        double sign = (data_in.wrench.force.x > 0) ? 1 : -1;
        data_out.wrench.force.x = data_in.wrench.force.x-threshold_lin_*sign;
    }
    else
    {
        data_out.wrench.force.x = 0;
    }
    if (fabs(data_in.wrench.force.y) > threshold_lin_)
    {
        double sign = (data_in.wrench.force.y > 0) ? 1 : -1;
        data_out.wrench.force.y = data_in.wrench.force.y-threshold_lin_*sign;
    }
    else
    {
        data_out.wrench.force.y = 0;
    }
    if (fabs(data_in.wrench.force.z) > threshold_lin_)
    {
        double sign = (data_in.wrench.force.z > 0) ? 1 : -1;
        data_out.wrench.force.z = data_in.wrench.force.z-threshold_lin_*sign;
    }
    else
    {
        data_out.wrench.force.z = 0;
    }
    if (fabs(data_in.wrench.torque.x) > threshold_angular_)
    {
        double sign = (data_in.wrench.torque.x > 0) ? 1 : -1;
        data_out.wrench.torque.x = data_in.wrench.torque.x-threshold_angular_*sign;
    }
    else
    {
        data_out.wrench.torque.x = 0;
    }
    if (fabs(data_in.wrench.torque.y) > threshold_angular_)
    {
        double sign = (data_in.wrench.force.y > 0) ? 1 : -1;
        data_out.wrench.torque.y = data_in.wrench.torque.y-threshold_angular_*sign;
    }
    else
    {
        data_out.wrench.torque.y = 0;
    }
    if (fabs(data_in.wrench.torque.z) > threshold_angular_)
    {
        double sign = (data_in.wrench.torque.z > 0) ? 1 : -1;
        data_out.wrench.torque.z = data_in.wrench.torque.z-threshold_angular_*sign;
    }
    else
    {
        data_out.wrench.torque.z = 0;
    }
  return true;
}

template <typename T>
void ThresholdFilter<T>::reconfigureConfigurationRequest(iirob_filters::ThresholdConfig& config, uint32_t level)
{
    //params_.fromConfig(config);
    threshold_ = params_.threshold;
    threshold_lin_ = params_.linear_threshold;
    threshold_angular_ = params_.angular_threshold;
};

template <typename T>
class MultiChannelThresholdFilter: public filters::MultiChannelFilterBase<T>
{
public:    
  MultiChannelThresholdFilter();
  ~MultiChannelThresholdFilter();
  
  virtual bool configure();
  virtual bool update(const std::vector<T>& data_in, std::vector<T>& data_out);
  
private:

    //ROS Objects
    iirob_filters::ThresholdParameters params_;
    double threshold_;
    double threshold_lin_;
    double threshold_angular_;
    
    using filters::MultiChannelFilterBase<T>::number_of_channels_;
};

template <typename T>
MultiChannelThresholdFilter<T>::MultiChannelThresholdFilter(): params_{ros::NodeHandle("~/ThresholdFilter/params").getNamespace()}
{
}

template <typename T>
MultiChannelThresholdFilter<T>::~MultiChannelThresholdFilter()
{
}


template <typename T>
bool MultiChannelThresholdFilter<T>::configure()
{
    params_.fromParamServer();
    threshold_ = params_.threshold;
    threshold_lin_ = params_.linear_threshold;
    threshold_angular_ = params_.angular_threshold;
    if(threshold_lin_ == 0)
	  ROS_ERROR("ThresholdFilter did not find param linear_threshold");
    if(threshold_angular_ == 0)
	  ROS_ERROR("ThresholdFilter did not find param angular_threshold");
    
  return true;
}

template <typename T>
bool MultiChannelThresholdFilter<T>::update(const std::vector<T>& data_in, std::vector<T>& data_out)
{
 
      
    if (data_in.size() != number_of_channels_ || data_out.size() != number_of_channels_)
    {
      ROS_ERROR("Configured with wrong size config:%d in:%d out:%d", number_of_channels_, (int)data_in.size(), (int)data_out.size());
      return false;
    }
    
    for(int i = 0; i < data_in.size(); i++)
    {
        data_out[i] = data_in[i];
        if (fabs(data_in[i]) > threshold_) {
             double sign = (data_in[i] > 0) ? 1 : -1;
             data_out[i] = threshold_*sign;
        }
        else
        {
            data_out[i] = 0;
        }
    }
    return true;
};
}
#endif
