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

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <iirob_filters/LowPassFilterParameters.h>
#include <iirob_filters/LowPassFilterConfig.h>
#include <dynamic_reconfigure/server.h>
#include <filters/filter_base.h>
#include <iirob_filters/iirob_filter_base.h>

#include <math.h>

namespace iirob_filters{
template <typename T>
class LowPassFilter: public iirob_filters::iirobFilterBase<T>
{
public:
        LowPassFilter();

        ~LowPassFilter();
        virtual bool configure();
        virtual bool configure(std::string ns);
        virtual bool update(const T& data_in, T& data_out);
        
private:

        ros::NodeHandle nh_;

        // Parameters
        double sampling_frequency_;
        double damping_frequency_;
        double damping_intensity_;
        int divider_;
        std::map<std::string,std::string> map_param_;


        // Filter parametrs
        double b1;
        double a1;
        int divider_counter;
        double filtered_value, filtered_old_value, old_value;
        std::string ns_;

        Eigen::Matrix<double,6,1> msg_filtered, msg_filtered_old, msg_old;
          
        dynamic_reconfigure::Server<iirob_filters::LowPassFilterConfig> reconfigCalibrationSrv_; // Dynamic reconfiguration service        

        void reconfigureConfigurationRequest(iirob_filters::LowPassFilterConfig& config, uint32_t level);
  
};


template <typename T>
LowPassFilter<T>::LowPassFilter()
{
    reconfigCalibrationSrv_.setCallback(boost::bind(&LowPassFilter<T>::reconfigureConfigurationRequest, this, _1, _2));
}

template <typename T>
LowPassFilter<T>::~LowPassFilter()
{
}
template <typename T>
bool LowPassFilter<T>::configure(std::string ns)
{
    ns_ = ns;    
    iirob_filters::LowPassFilterParameters params_{ns_+"/params"};
    params_.fromParamServer();
    sampling_frequency_ = params_.SamplingFrequency;
    damping_frequency_ = params_.DampingFrequency;
    damping_intensity_ = params_.DampingIntensity;

    
    divider_ = params_.divider;    
    if(sampling_frequency_ == 0)
        ROS_ERROR("LowPassFilter did not find param SamplingFrequency");
    if(damping_frequency_ == 0)
        ROS_ERROR("LowPassFilter did not find param DampingFrequency");
    if(damping_intensity_ == 0)
        ROS_ERROR("LowPassFilter did not find param DampingIntensity");
    if(divider_ == 0)
        ROS_ERROR("Divider value not correct - cannot be 0. Check .param or .yaml files");
    
    a1 = exp(-1 / sampling_frequency_ * (2 * M_PI * damping_frequency_) / (pow(10, damping_intensity_ / -10.0)));
    b1 = 1 - a1;
    
    divider_counter = 1;
    // Initialize storage Vectors
    filtered_value = filtered_old_value = old_value = 0;
    for (int ii=0; ii<6; ii++)
    {
        msg_filtered(ii) = msg_filtered_old(ii) = msg_old(ii) = 0;
    }
    return true;
}

template <typename T>
bool LowPassFilter<T>::configure()
{    
    iirob_filters::LowPassFilterParameters params_{nh_.getNamespace()+"/LowPassFilter/params"};
    params_.fromParamServer();
    sampling_frequency_ = params_.SamplingFrequency;
    damping_frequency_ = params_.DampingFrequency;
    damping_intensity_ = params_.DampingIntensity;
    divider_ = params_.divider;    
    if(sampling_frequency_ == 0)
        ROS_ERROR("LowPassFilter did not find param SamplingFrequency");
    if(damping_frequency_ == 0)
        ROS_ERROR("LowPassFilter did not find param DampingFrequency");
    if(damping_intensity_ == 0)
        ROS_ERROR("LowPassFilter did not find param DampingIntensity");
    if(divider_ == 0)
        ROS_ERROR("Divider value not correct - cannot be 0. Check .param or .yaml files");
    
    a1 = exp(-1 / sampling_frequency_ * (2 * M_PI * damping_frequency_) / (pow(10, damping_intensity_ / -10.0)));
    b1 = 1 - a1;
    
    divider_counter = 1;
    // Initialize storage Vectors
    filtered_value = filtered_old_value = old_value = 0;
    for (int ii=0; ii<6; ii++)
    {
        msg_filtered(ii) = msg_filtered_old(ii) = msg_old(ii) = 0;
    }
    return true;
}

template<>
inline bool LowPassFilter<geometry_msgs::WrenchStamped>::update(const geometry_msgs::WrenchStamped& data_in, geometry_msgs::WrenchStamped& data_out){    
    // IIR Filter
    msg_filtered = b1 * msg_old + a1 * msg_filtered_old;
    msg_filtered_old = msg_filtered;    

    //TODO use wrenchMsgToEigen
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

template<typename T>
bool LowPassFilter<T>::update(const T& data_in, T& data_out)
{    
    data_out = b1 * old_value + a1 * filtered_old_value;
    filtered_old_value = data_out;
    old_value = data_in;
    
    return true;
}

template <typename T>
void LowPassFilter<T>::reconfigureConfigurationRequest(iirob_filters::LowPassFilterConfig& config, uint32_t level)
{
    iirob_filters::LowPassFilterParameters params_{ns_+"/params"};
    //params_.fromConfig(config);
    sampling_frequency_ = params_.SamplingFrequency;
    damping_intensity_ = params_.DampingFrequency;
    damping_intensity_ = params_.DampingIntensity;
};

/* A lp filter which works on double arrays.
 *
 */
template <typename T>
class MultiChannelLowPassFilter: public filters::MultiChannelFilterBase <T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  MultiChannelLowPassFilter();

  /** \brief Destructor to clean up
   */
  ~MultiChannelLowPassFilter();

  virtual bool configure();

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update( const std::vector<T> & data_in, std::vector<T>& data_out);
  
protected:
  
  ros::NodeHandle nh_;
  
  double sampling_frequency_;
  double damping_frequency_;
  double damping_intensity_;
  int divider_;
  
  // Filter parametrs
  int divider_counter;
  double b1;
  double a1;
  
  iirob_filters::LowPassFilterParameters params_;  
  std::vector<T> filtered_value, filtered_old_value, old_value;
  
  using filters::MultiChannelFilterBase<T>::number_of_channels_;           
};


template <typename T>
MultiChannelLowPassFilter<T>::MultiChannelLowPassFilter():params_{nh_.getNamespace()+"/LowPassFilter/params"}
{
}

template <typename T>
MultiChannelLowPassFilter<T>::~MultiChannelLowPassFilter()
{
}

template <typename T>
bool MultiChannelLowPassFilter<T>::configure()
{
    params_.fromParamServer();
    sampling_frequency_ = params_.SamplingFrequency;
    damping_frequency_ = params_.DampingFrequency;
    damping_intensity_ = params_.DampingIntensity;
    divider_ = params_.divider;
    if(sampling_frequency_ == 0)
        ROS_ERROR("MultiChannelLowPassFilter did not find param SamplingFrequency");
    if(damping_frequency_ == 0)
        ROS_ERROR("MultiChannelLowPassFilter did not find param DampingFrequency");
    if(damping_intensity_ == 0)
        ROS_ERROR("MultiChannelLowPassFilter did not find param DampingIntensity");
    if(divider_ == 0)
        ROS_ERROR("Divider value not correct - cannot be 0. Check .param or .yaml files");
    
    a1 = exp(-1 / sampling_frequency_ * (2 * M_PI * damping_frequency_) / (pow(10, damping_intensity_ / -10.0)));
    b1 = 1 - a1;
    divider_counter = 1;
    
    // Initialize storage Vectors
    
    for (int ii=0; ii<number_of_channels_; ii++)
    {
        filtered_value[ii] = filtered_old_value[ii] = old_value[ii] = 0;
    }
  return true;
}


template <typename T>
bool MultiChannelLowPassFilter<T>::update(const std::vector<T> & data_in, std::vector<T>& data_out)
{

  if (data_in.size() != number_of_channels_ || data_out.size() != number_of_channels_)
  {
    ROS_ERROR("Configured with wrong size config:%d in:%d out:%d", number_of_channels_, (int)data_in.size(), (int)data_out.size());
    return false;
  }
  for (int i = 0; i<data_in.size();i++)
  {
    data_out[i] = b1 * old_value[i] + a1 * filtered_old_value[i];
    filtered_old_value[i] = data_out[i];
  }
  old_value = data_in;
  return true;
    
};

}
#endif
