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
#include <filters/filter_base.h>

#include <math.h>

namespace iirob_filters{
template <typename T>
class LowPassFilter: public filters::FilterBase<T>
{
public:
        LowPassFilter(double sampling_frequency = 0.0, double damping_frequency = 0.0, double damping_intensity = 0.0, double divider = 0.0);
        //LowPassFilter();

        ~LowPassFilter();
        virtual bool configure();
        virtual bool update(const T & data_in, T& data_out);
        virtual bool update(const geometry_msgs::WrenchStamped& to_filter_wrench, geometry_msgs::WrenchStamped& filtered_wrench);

private:

        ros::NodeHandle nh_;

        // Parameters
        double sampling_frequency_;
        double damping_frequency_;
        double damping_intensity_;
        int divider_;


        // Filter parametrs
        double b1;
        double a1;
        int divider_counter;
        iirob_filters::LowPassFilterParameters params_;
        double filtered_value, filtered_old_value, old_value, mean_value;

        Eigen::Matrix<double,6,1> msg_filtered, msg_filtered_old, msg_old, wrench_mean;
};

template <typename T>
LowPassFilter<T>::LowPassFilter(double sampling_frequency, double damping_frequency, double damping_intensity, double divider)
    : sampling_frequency_(sampling_frequency), damping_frequency_(damping_frequency), damping_intensity_(damping_intensity), divider_(divider), params_{nh_.getNamespace()+"/LowPassFilter/Force_x"}
{

}

/*template <typename T>
LowPassFilter<T>::LowPassFilter(): params_{nh_.getNamespace()+"/LowPassFilter"}
{
}*/

template <typename T>
LowPassFilter<T>::~LowPassFilter()
{
}

template <typename T>
bool LowPassFilter<T>::configure()
{
    params_.fromParamServer();
    sampling_frequency_ = params_.SamplingFrequency;
    damping_frequency_ = params_.DampingFrequency;
    damping_intensity_ = params_.DampingIntensity;
    
    a1 = exp(-1 / sampling_frequency_ * (2 * M_PI * damping_frequency_) / (pow(10, damping_intensity_ / -10.0)));
    b1 = 1 - a1;

    divider_counter = 1;
    // Initialize storage Vectors
    filtered_value = filtered_old_value = old_value = mean_value = 0;
    for (int ii=0; ii<6; ii++)
    {
        msg_filtered(ii) = msg_filtered_old(ii) = msg_old(ii) = wrench_mean(ii) = 0;
    }
    return true;
}

template <typename T>
bool LowPassFilter<T>::update(const T & data_in, T& data_out)
{
    // IIR Filter
    data_out = b1 * old_value + a1 * filtered_old_value;
    filtered_old_value = data_out;

    old_value = data_in;

    return true;
}

template <typename T>
bool LowPassFilter<T>::update(const geometry_msgs::WrenchStamped& to_filter_wrench, geometry_msgs::WrenchStamped& filtered_wrench)
{    
      // IIR Filter
    msg_filtered = b1 * msg_old + a1 * msg_filtered_old;
    msg_filtered_old = msg_filtered;

    //TODO use wrenchMsgToEigen
    msg_old[0] = to_filter_wrench.wrench.force.x;
    msg_old[1] = to_filter_wrench.wrench.force.y;
    msg_old[2] = to_filter_wrench.wrench.force.z;
    msg_old[3] = to_filter_wrench.wrench.torque.x;
    msg_old[4] = to_filter_wrench.wrench.torque.y;
    msg_old[5] = to_filter_wrench.wrench.torque.z;

    // Mean Filter
    wrench_mean += msg_filtered;
    if (divider_counter < divider_)
    {
        divider_counter++;
    }
    else
    {
        wrench_mean /= divider_;
        divider_counter = 1;


        //geometry_msgs::WrenchStamped filtered_wrench;
        //TODO use wrenchEigenToMsg
        filtered_wrench.wrench.force.x = wrench_mean[0];
        filtered_wrench.wrench.force.y = wrench_mean[1];
        filtered_wrench.wrench.force.z = wrench_mean[2];
        filtered_wrench.wrench.torque.x = wrench_mean[3];
        filtered_wrench.wrench.torque.y = wrench_mean[4];
        filtered_wrench.wrench.torque.z = wrench_mean[5];

        filtered_wrench.header = to_filter_wrench.header;
        wrench_mean.setZero();

   }
   return true;
};


/* A lp filter which works on double arrays.
 *
 */
template <typename T>
class MultiChannelLowPassFilter: public filters::MultiChannelFilterBase <T>
{
public:
  /** \brief Construct the filter with the expected width and height */
  MultiChannelLowPassFilter(double divider = 1.0);

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
  //boost::scoped_ptr<RealtimeCircularBuffer<std::vector<T> > > data_storage_; ///< Storage for data between updates
  //uint32_t last_updated_row_;                     ///< The last row to have been updated by the filter

  //std::vector<T> temp;  //used for preallocation and copying from non vector source

  //uint32_t number_of_observations_;             ///< Number of observations over which to filter
  
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
  double filtered_value, filtered_old_value, old_value, mean_value;
  Eigen::Matrix<double,6,1> msg_filtered, msg_filtered_old, msg_old, wrench_mean;
  
  using filters::MultiChannelFilterBase<T>::number_of_channels_;           ///< Number of elements per observation  
};


template <typename T>
MultiChannelLowPassFilter<T>::MultiChannelLowPassFilter(double divider):
 params_{nh_.getNamespace()+"/LowPassFilter/Force_x"}, divider_(divider)
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
    std::cout<<sampling_frequency_<<" "<<damping_frequency_<<" "<<damping_intensity_<<std::endl;
    
    a1 = exp(-1 / sampling_frequency_ * (2 * M_PI * damping_frequency_) / (pow(10, damping_intensity_ / -10.0)));
    b1 = 1 - a1;
    std::cout<<"a b"<<a1<<" "<<b1<<std::endl;
    divider_counter = 1;
    // Initialize storage Vectors
    filtered_value = filtered_old_value = old_value = mean_value = 0;
    for (int ii=0; ii<6; ii++)
    {
        msg_filtered(ii) = msg_filtered_old(ii) = msg_old(ii) = wrench_mean(ii) = 0;
    }
    //std::cout<<"wr:"<<wrench_mean<<std::endl;
  return true;
}


template <typename T>
bool MultiChannelLowPassFilter<T>::update(const std::vector<T> & data_in, std::vector<T>& data_out)
{
  //  ROS_ASSERT(data_in.size() == width_);
  //ROS_ASSERT(data_out.size() == width_);
  if (data_in.size() != number_of_channels_ || data_out.size() != number_of_channels_)
  {
    ROS_ERROR("Configured with wrong size config:%d in:%d out:%d", number_of_channels_, (int)data_in.size(), (int)data_out.size());
    return false;
  }
    geometry_msgs::WrenchStamped to_filter_wrench;
    to_filter_wrench.wrench.force.x = data_in.at(0);
    to_filter_wrench.wrench.force.y = data_in.at(1);
    to_filter_wrench.wrench.force.z = data_in.at(2);
    to_filter_wrench.wrench.torque.x = data_in.at(3);
    to_filter_wrench.wrench.torque.y = data_in.at(4);
    to_filter_wrench.wrench.torque.z = data_in.at(5);
    // IIR Filter
    msg_filtered = b1 * msg_old + a1 * msg_filtered_old;
    msg_filtered_old = msg_filtered;

    //TODO use wrenchMsgToEigen
    msg_old[0] = to_filter_wrench.wrench.force.x;
    msg_old[1] = to_filter_wrench.wrench.force.y;
    msg_old[2] = to_filter_wrench.wrench.force.z;
    msg_old[3] = to_filter_wrench.wrench.torque.x;
    msg_old[4] = to_filter_wrench.wrench.torque.y;
    msg_old[5] = to_filter_wrench.wrench.torque.z;

    // Mean Filter
    wrench_mean += msg_filtered;
    if (divider_counter < divider_)
    {
        divider_counter++;
    }
    else
    {
        wrench_mean /= divider_;
        divider_counter = 1;
        //std::cout<<"divider "<<divider_<<" "<<wrench_mean<<std::endl;

        geometry_msgs::WrenchStamped filtered_wrench;
        //TODO use wrenchEigenToMsg
        filtered_wrench.wrench.force.x = wrench_mean[0];
        filtered_wrench.wrench.force.y = wrench_mean[1];
        filtered_wrench.wrench.force.z = wrench_mean[2];
        filtered_wrench.wrench.torque.x = wrench_mean[3];
        filtered_wrench.wrench.torque.y = wrench_mean[4];
        filtered_wrench.wrench.torque.z = wrench_mean[5];

        filtered_wrench.header = to_filter_wrench.header;
        wrench_mean.setZero();
        //std::cout<<"filtered "<<filtered_wrench.wrench.force.x<<std::endl;
        data_out.clear();
        data_out.push_back(filtered_wrench.wrench.force.x);
        data_out.push_back(filtered_wrench.wrench.force.y);
        data_out.push_back(filtered_wrench.wrench.force.z);
        data_out.push_back(filtered_wrench.wrench.torque.x);
        data_out.push_back(filtered_wrench.wrench.torque.y);
        data_out.push_back(filtered_wrench.wrench.torque.z);          
   }
   //std::cout<<"out:"<<data_out<<std::endl;

  return true;
};

}
#endif
