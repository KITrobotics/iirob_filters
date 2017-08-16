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

class ThresholdFilter
{

private:
  ros::NodeHandle nh_;
  iirob_filters::ThresholdParameters params_;
  double threshold_;
  double threshold_lin_;
  double threshold_angular_;

public:
  ThresholdFilter();
  ThresholdFilter(ros::NodeHandle nh);

  ThresholdFilter(double threshold);

  ThresholdFilter(double threshold_lin, double threshold_angular);

  bool init(const ros::NodeHandle &nh);


  double applyFilter(double value);

  geometry_msgs::WrenchStamped applyFilter(const geometry_msgs::WrenchStamped& to_filter_wrench);
};

#endif
