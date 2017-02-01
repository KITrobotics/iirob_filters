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

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

class GravityCompensator
{

private:

  //ROS Objects
  ros::NodeHandle nh_;

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

  bool init();

public:

  GravityCompensator(ros::NodeHandle& nh);
  GravityCompensator();
  bool init(const ros::NodeHandle &nh);

  GravityCompensator(std::string world_frame, double cog_x, double cog_y, double cog_z, double force_z);

  geometry_msgs::WrenchStamped compensate(const geometry_msgs::WrenchStamped &to_compensate_wrench);

};

#endif
