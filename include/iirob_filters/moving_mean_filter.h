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

#ifndef IIROB_FILTERS_MOVING_MEAN_FILTER_H
#define IIROB_FILTERS_MOVING_MEAN_FILTER_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iirob_filters/MovingMeanParameters.h>
#include <filters/filter_base.h>
#include <math.h>
namespace iirob_filters{
    
template <typename T>
class MovingMeanFilter: public filters::FilterBase<T>
{
public:

        MovingMeanFilter();
        ~MovingMeanFilter();

        virtual bool configure();
        virtual bool update(const T & data_in, T& data_out);

private:

        ros::NodeHandle nh_;
        
        // Parameters
        iirob_filters::MovingMeanParameters params_;
        int divider_;

        // Filter parametrs
        int divider_counter;
        std::vector<T> values;
};
    
template <typename T>
MovingMeanFilter<T>::MovingMeanFilter():params_{nh_.getNamespace()+"/MovingMeanFilter/params"}
{
}

template <typename T>
MovingMeanFilter<T>::~MovingMeanFilter()
{
}

template <typename T>
bool MovingMeanFilter<T>::configure()
{
    params_.fromParamServer();
    divider_ = params_.divider;

    ROS_INFO("Moving Mean Filter Params: Divider: %d " , divider_);

    if(divider_ == 0)
        ROS_ERROR("MovingMeanFilter did not find param divider");
    return true;
}

template <typename T>
bool MovingMeanFilter<T>::update(const T & data_in, T& data_out)
{
  if (values.size() < divider_) {
    values.push_back(data_in);
    data_out = data_in;
    return true;
  }

  values.erase(values.begin());
  values.push_back(data_in);

  T sum;
  sum.wrench.force.x = 0.0;
  sum.wrench.force.y = 0.0;
  sum.wrench.force.z = 0.0;
  sum.wrench.torque.x = 0.0;
  sum.wrench.torque.y = 0.0;
  sum.wrench.torque.x = 0.0;  
  for(int i =0 ; i<values.size(); ++i) 
  {
    sum.wrench.force.x += values[i].wrench.force.x;
    sum.wrench.force.y += values[i].wrench.force.y;
    sum.wrench.force.z += values[i].wrench.force.z;
    sum.wrench.torque.x += values[i].wrench.torque.x;
    sum.wrench.torque.y += values[i].wrench.torque.y;
    sum.wrench.torque.z += values[i].wrench.torque.z;
    
  }
  data_out.wrench.force.x = sum.wrench.force.x / values.size();
  data_out.wrench.force.y = sum.wrench.force.y / values.size();
  data_out.wrench.force.z = sum.wrench.force.z / values.size();
  data_out.wrench.torque.x = sum.wrench.torque.x / values.size();
  data_out.wrench.torque.y = sum.wrench.torque.y / values.size();
  data_out.wrench.torque.z = sum.wrench.torque.z / values.size();
  
  return true;
};
}

#endif

 
