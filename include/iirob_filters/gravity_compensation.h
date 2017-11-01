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
#include <iirob_filters/FilterInterface.h>
#include <filters/filter_base.h>

namespace iirob_filters{

template <typename T>
class GravityCompensator: public filters::FilterBase<T>
{
public:
    GravityCompensator();
    GravityCompensator(ros::NodeHandle& nh);
    GravityCompensator(std::string world_frame, double cog_x, double cog_y, double cog_z, double force_z);    
      
    /** \brief Destructor to clean up
    */
    ~GravityCompensator();
    
    virtual bool configure();
    
    /** \brief Update the filter and return the data seperately
    * \param data_in T array with length width
    * \param data_out T array with length width
    */
    virtual bool update( const T & data_in, T& data_out);
    
    virtual bool update(const geometry_msgs::WrenchStamped& to_compensate_wrench, geometry_msgs::WrenchStamped& compensated_wrench);

private:

    //ROS Objects
    ros::NodeHandle nh_;
    iirob_filters::GravityCompensationParameters params_;

    // Storage for Calibration Values
    geometry_msgs::Vector3Stamped cog_; // Center of Gravity Vector (wrt Sensor Frame)
    double force_z_; // Gravitational Force

    // Frames for Transformation of Gravity / CoG Vector
    std::string world_frame_;

    // tf2 objects
    tf2_ros::Buffer *p_tf_Buffer_;
    tf2_ros::TransformListener *p_tf_Listener;
    geometry_msgs::TransformStamped transform_, transform_back_;
  
    uint _num_transform_errors;
    
    //bool init();
};

template <typename T>
GravityCompensator<T>::GravityCompensator(ros::NodeHandle& nh) :
  nh_(nh), params_{nh_.getNamespace()+"/GravityCompensation"}
{
 /* params_.fromParamServer();
  cog_.vector.x = params_.CoG_x;
  cog_.vector.y = params_.CoG_y;
  cog_.vector.z = params_.CoG_z;
  force_z_ = params_.force;
  world_frame_ = params_.world_frame;
  init();*/
}

template <typename T>
GravityCompensator<T>::GravityCompensator(): params_{nh_.getNamespace()+"/GravityCompensation"}
{
}

template <typename T>
GravityCompensator<T>::GravityCompensator(std::string world_frame, double cog_x, double cog_y, double cog_z, double force_z)
  : world_frame_(world_frame), force_z_(force_z), params_{nh_.getNamespace()+"/GravityCompensation"}
{
  cog_.vector.x = cog_x;
  cog_.vector.y = cog_y;
  cog_.vector.z = cog_z;

  //init();
}

template <typename T>
GravityCompensator<T>::~GravityCompensator()
{
}

template <typename T>
bool GravityCompensator<T>::configure()
{
    //nh.param<std::string>("default_param", default_param, "default_value"); default value version
    /*std::vector<double> cog;    
    if(nh_.hasParam("CoG"))
    {
        if (!filters::FilterBase<T>::getParam(std::string("CoG"),cog))
        {
            ROS_ERROR("GravityCompensator did not find param CoG");
            return false;
        }    
    }
    else
    {
        cog_.vector.x = 0;
        cog_.vector.y = 0;
        cog_.vector.z = 0;
    }

    if(nh_.hasParam("force"))
    {
        if (!filters::FilterBase<T>::getParam(std::string("force"),force_z_))
        {
            ROS_ERROR("GravityCompensator did not find param force");
            return false;
        }
    }
    else force_z_ = 0;
    std::cout<<"param: "<<nh_.hasParam("fts/GravityCompensation/world_frame")<<std::endl;
    if(nh_.hasParam("world_frame"))
    {
        if (!filters::FilterBase<T>::getParam(std::string("world_frame"),world_frame_))
        {
            ROS_ERROR("GravityCompensator did not find param world_frame");
            return false;
        }
    }
    else
    {
     world_frame_=" ";   
    }*/
    params_.fromParamServer();
    cog_.vector.x = params_.CoG_x;
    cog_.vector.y = params_.CoG_y;
    cog_.vector.z = params_.CoG_z;
    force_z_ = params_.force;
    world_frame_ = params_.world_frame;
    
    p_tf_Buffer_ = new tf2_ros::Buffer;
    p_tf_Listener = new tf2_ros::TransformListener(*p_tf_Buffer_,true);
    
    _num_transform_errors = 0;

    //init();
    return true;
}
template <typename T>
bool GravityCompensator<T>::update( const T & data_in, T& data_out)
{
    return true;
}

template <typename T>
bool GravityCompensator<T>::update(const geometry_msgs::WrenchStamped& to_compensate_wrench, geometry_msgs::WrenchStamped& compensated_wrench)
{
  // Optimise this to get transform only once
  try
  {
    transform_ = p_tf_Buffer_->lookupTransform(world_frame_, to_compensate_wrench.header.frame_id, ros::Time(0));
    transform_back_ = p_tf_Buffer_->lookupTransform(to_compensate_wrench.header.frame_id, world_frame_, ros::Time(0));
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

  temp_vector_in.vector = to_compensate_wrench.wrench.force;
  tf2::doTransform(temp_vector_in, temp_force_transformed, transform_);

  temp_vector_in.vector = to_compensate_wrench.wrench.torque;
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
  compensated_wrench = to_compensate_wrench;
  
  tf2::doTransform(temp_force_transformed, temp_vector_out, transform_back_);
  compensated_wrench.wrench.force = temp_vector_out.vector;

  tf2::doTransform(temp_torque_transformed, temp_vector_out, transform_back_);
  compensated_wrench.wrench.torque = temp_vector_out.vector;

  return true;
}
}
#endif
