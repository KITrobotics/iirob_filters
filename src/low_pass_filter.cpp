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

#include <iirob_filters/low_pass_filter.h>

LowPassFilter::LowPassFilter(double sampling_frequency, double damping_frequency, double damping_intensity, double divider)
    : sampling_frequency_(sampling_frequency), damping_frequency_(damping_frequency), damping_intensity_(damping_intensity), divider_(divider), params_{ros::NodeHandle(nh_)}
{
    init();
}

bool LowPassFilter::init()
{
    a1 = exp(-1 / sampling_frequency_ * (2 * M_PI * damping_frequency_) / (pow(10, damping_intensity_ / -10.0)));
    b1 = 1 - a1;

    divider_counter = 1;
    // Initialize storage Vectors
    filtered_value = filtered_old_value = old_value = mean_value = 0;
    for (int ii=0; ii<6; ii++)
    {
        msg_filtered(ii) = msg_filtered_old(ii) = msg_old(ii) = wrench_mean(ii) = 0;
    }
}

bool LowPassFilter::init(const ros::NodeHandle &nh)
{
    params_.setNamespace(nh.getNamespace());
    params_.fromParamServer();
    sampling_frequency_ = params_.SamplingFrequency;
    damping_frequency_ = params_.DampingFrequency;
    damping_intensity_ = params_.DampingIntensity;
    init();
}

double LowPassFilter::applyFilter(double value)
{
    // IIR Filter
    filtered_value = b1 * old_value + a1 * filtered_old_value;
    filtered_old_value = filtered_value;

    old_value = value;

    return filtered_value;
}

geometry_msgs::WrenchStamped LowPassFilter::applyFilter(geometry_msgs::WrenchStamped& to_filter_wrench)
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

        return filtered_wrench;
    }
}
