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

#include <iirob_filters/gravity_compensation.h>

GravityCompensator::GravityCompensator(ros::NodeHandle& nh) :
  nh_(nh)
{
  nh_.param<double>("CoG/x", cog_.vector.x, 0.0);
  nh_.param<double>("CoG/y", cog_.vector.y, 0.0);
  nh_.param<double>("CoG/z", cog_.vector.z, 0.0);
  nh_.param<double>("force", force_z_, 0.0);
  nh_.param<std::string>("sensor_frame", sensor_frame_, "");
  nh_.param<std::string>("world_frame", world_frame_, "");

  init();
}
GravityCompensator::GravityCompensator() {}

GravityCompensator::GravityCompensator(std::string sensor_frame, std::string world_frame, double cog_x, double cog_y, double cog_z, double force_z)
  : sensor_frame_(sensor_frame), world_frame_(world_frame), force_z_(force_z)
{
  cog_.vector.x = cog_x;
  cog_.vector.y = cog_y;
  cog_.vector.z = cog_z;

  init();
}

bool GravityCompensator::init()
{
  // Construct ROS Objects
  p_tf_Buffer_ = new tf2_ros::Buffer;
  p_tf_Listener = new tf2_ros::TransformListener(*p_tf_Buffer_,true);

  return true;
}

bool GravityCompensator::init(const ros::NodeHandle &nh)
{
    nh.param<double>("CoG/x", cog_.vector.x, 0.0);
    nh.param<double>("CoG/y", cog_.vector.y, 0.0);
    nh.param<double>("CoG/z", cog_.vector.z, 0.0);
    nh.param<double>("force", force_z_, 0.0);
    nh.param<std::string>("sensor_frame", sensor_frame_, "");
    nh.param<std::string>("world_frame", world_frame_, "");
    return init();
}

geometry_msgs::WrenchStamped GravityCompensator::compensate(const geometry_msgs::WrenchStamped& to_compensate_wrench)
{
  // Optimise this to get transform only once
  try
  {
    transform_cog_ = p_tf_Buffer_->lookupTransform(world_frame_, sensor_frame_, ros::Time(0));
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  // Transform CoG Vector
  geometry_msgs::Vector3Stamped cog_transformed;
  tf2::doTransform(cog_, cog_transformed, transform_cog_);

  // Copy Message and Compensate values for Gravity Force and Resulting Torque
  geometry_msgs::WrenchStamped compensated_wrench;
  compensated_wrench = to_compensate_wrench;

  // Compensate for gravity force
  compensated_wrench.wrench.force.z  += force_z_;
  // Compensation Values for torque result from Crossprod of cog Vector and (0 0 G)
  compensated_wrench.wrench.torque.x += (force_z_ * cog_transformed.vector.y);
  compensated_wrench.wrench.torque.y -= (force_z_ * cog_transformed.vector.x);

  return compensated_wrench;
}
