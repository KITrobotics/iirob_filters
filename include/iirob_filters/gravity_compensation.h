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
#include <stdint.h>
#include <cstring>
#include <stdio.h>

#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <iirob_filters/GravityCompensationParameters.h>
#include <iirob_filters/GravityCompensationConfig.h>
#include <dynamic_reconfigure/server.h>
#include <filters/filter_base.h>
#include <iirob_filters/iirob_filter_base.h>

namespace iirob_filters{

template <typename T>
class GravityCompensator: public iirob_filters::iirobFilterBase<T>
{
public:
    GravityCompensator();
      
    /** \brief Destructor to clean up
    */
    ~GravityCompensator();
    
    virtual bool configure();
    virtual bool configure(std::string ns);
    
    /** \brief Update the filter and return the data seperately
    * \param data_in T array with length width
    * \param data_out T array with length width
    */
    virtual bool update( const T & data_in, T& data_out);

private:
    ros::NodeHandle nh_;
    std::string ns_;
    //iirob_filters::GravityCompensationParameters params_;

    // Storage for Calibration Values
    geometry_msgs::Vector3Stamped cog_; // Center of Gravity Vector (wrt Sensor Frame)
    double force_z_; // Gravitational Force

    // Frames for Transformation of Gravity / CoG Vector
    std::string world_frame_;
    std::string sensor_frame_;

    // tf2 objects
    tf2_ros::Buffer *p_tf_Buffer_;
    tf2_ros::TransformListener *p_tf_Listener;
    geometry_msgs::TransformStamped transform_, transform_back_;
  
    uint _num_transform_errors;
    
                      
    dynamic_reconfigure::Server<iirob_filters::GravityCompensationConfig> reconfigCalibrationSrv_; // Dynamic reconfiguration service        

    void reconfigureConfigurationRequest(iirob_filters::GravityCompensationConfig& config, uint32_t level);
};

template <typename T>
GravityCompensator<T>::GravityCompensator()
{
    reconfigCalibrationSrv_.setCallback(boost::bind(&GravityCompensator<T>::reconfigureConfigurationRequest, this, _1, _2));
}

template <typename T>
GravityCompensator<T>::~GravityCompensator()
{
}

template <typename T>
bool GravityCompensator<T>::configure(std::string ns)
{
    ns_ = ns;    
    iirob_filters::GravityCompensationParameters params_{ns_+"/params"};
    params_.fromParamServer();
    if(params_.world_frame == " ")
      ROS_ERROR("GravityCompensator did not find param world_frame");
    if(params_.sensor_frame == " ")
      ROS_DEBUG("GravityCompensator did not find param sensor_frame");
    if(params_.CoG_x == 0)
      ROS_DEBUG("GravityCompensator did not find param CoG_x");
    if(params_.CoG_y == 0)
      ROS_DEBUG("GravityCompensator did not find param CoG_y");
    if(params_.CoG_z == 0)
      ROS_DEBUG("GravityCompensator did not find param CoG_z");
    if(params_.force == 0)
      ROS_DEBUG("GravityCompensator did not find param force");
    
    sensor_frame_ = params_.sensor_frame;
    cog_.vector.x = params_.CoG_x;
    cog_.vector.y = params_.CoG_y;
    cog_.vector.z = params_.CoG_z;
    force_z_ = params_.force;
    world_frame_ = params_.world_frame;    
    
    p_tf_Buffer_ = new tf2_ros::Buffer;
    p_tf_Listener = new tf2_ros::TransformListener(*p_tf_Buffer_,true);

    _num_transform_errors = 0;
    
    return true;
}

template <typename T>
bool GravityCompensator<T>::configure()
{
    std::cout<<"conf() gc"<<std::endl;
    iirob_filters::GravityCompensationParameters params_{nh_.getNamespace()+"/GravityCompensator/params"};
    params_.fromParamServer();
    if(params_.world_frame == " ")
      ROS_ERROR("GravityCompensator did not find param world_frame");
    if(params_.sensor_frame == " ")
      ROS_DEBUG("GravityCompensator did not find param sensor_frame");
    if(params_.CoG_x == 0)
      ROS_DEBUG("GravityCompensator did not find param CoG_x");
    if(params_.CoG_y == 0)
      ROS_DEBUG("GravityCompensator did not find param CoG_y");
    if(params_.CoG_z == 0)
      ROS_DEBUG("GravityCompensator did not find param CoG_z");
    if(params_.force == 0)
      ROS_DEBUG("GravityCompensator did not find param force");
    
    sensor_frame_ = params_.sensor_frame;
    cog_.vector.x = params_.CoG_x;
    cog_.vector.y = params_.CoG_y;
    cog_.vector.z = params_.CoG_z;
    force_z_ = params_.force;
    
    p_tf_Buffer_ = new tf2_ros::Buffer;
    p_tf_Listener = new tf2_ros::TransformListener(*p_tf_Buffer_,true);
    world_frame_ = params_.world_frame;
    _num_transform_errors = 0;
    
    return true;
}
template <typename T>
bool GravityCompensator<T>::update(const T & data_in, T& data_out)
{  
  try
  {
    transform_ = p_tf_Buffer_->lookupTransform(world_frame_, data_in.header.frame_id, ros::Time(0));
    transform_back_ = p_tf_Buffer_->lookupTransform(data_in.header.frame_id, world_frame_, ros::Time(0));
    _num_transform_errors = 0;
  }
  catch (tf2::TransformException ex)
  {
    if (_num_transform_errors%100 == 0){
      ROS_ERROR("%s", ex.what());
    }
    _num_transform_errors++;
  }
  
  geometry_msgs::Vector3Stamped temp_force_transformed, temp_torque_transformed, temp_vector_in, temp_vector_out;

  temp_vector_in.vector = data_in.wrench.force;
  tf2::doTransform(temp_vector_in, temp_force_transformed, transform_);

  temp_vector_in.vector = data_in.wrench.torque;
  tf2::doTransform(temp_vector_in, temp_torque_transformed, transform_);
  
  // Transform CoG Vector
  geometry_msgs::Vector3Stamped cog_transformed;
  tf2::doTransform(cog_, cog_transformed, transform_);
  
    // Compensate for gravity force
  temp_force_transformed.vector.z  += force_z_;
  // Compensation Values for torque result from Crossprod of cog Vector and (0 0 G)
  temp_torque_transformed.vector.x += (force_z_ * cog_transformed.vector.y);
  temp_torque_transformed.vector.y -= (force_z_ * cog_transformed.vector.x);

  // Copy Message and Compensate values for Gravity Force and Resulting Torque
  //geometry_msgs::WrenchStamped compensated_wrench;
  data_out = data_in;
  
  tf2::doTransform(temp_force_transformed, temp_vector_out, transform_back_);
  data_out.wrench.force = temp_vector_out.vector;

  tf2::doTransform(temp_torque_transformed, temp_vector_out, transform_back_);
  data_out.wrench.torque = temp_vector_out.vector;
  return true;
}

template <typename T>
void GravityCompensator<T>::reconfigureConfigurationRequest(iirob_filters::GravityCompensationConfig& config, uint32_t level)
{
    iirob_filters::GravityCompensationParameters params_{ns_+"/params"};
    //params_.fromConfig(config);
    world_frame_ = params_.world_frame;      
    sensor_frame_ = params_.sensor_frame;
    cog_.vector.x = params_.CoG_x;
    cog_.vector.y = params_.CoG_y;
    cog_.vector.z = params_.CoG_z;
    force_z_ = params_.force;
};

}
#endif

