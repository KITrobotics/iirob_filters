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

#ifndef IIROB_FILTERS_KALMAN_FILTER_H
#define IIROB_FILTERS_KALMAN_FILTER_H

#include <filters/filter_base.h>
#include <Eigen/Dense>
#include <iirob_filters/KalmanFilterParameters.h>
#include <ros/ros.h>



namespace iirob_filters {
template <typename T>
class MultiChannelKalmanFilter : public filters::MultiChannelFilterBase<T>
{
public:
  MultiChannelKalmanFilter();
  ~MultiChannelKalmanFilter();
  virtual bool configure();
  virtual bool update(const std::vector<T>& data_in, std::vector<T>& data_out);
  
private:
    // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, P, K, P0;

  // System dimensions
  int m, n;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
  
  ros::NodeHandle nh_;
  
  iirob_filters::KalmanFilterParameters params_; 
  
  bool fromStdVectorToEigenMatrix(std::vector<double>& in, Eigen::MatrixXd& out, int rows, int columns, std::string matrix_name);
  bool fromStdVectorToEigenVector(std::vector<double>& in, Eigen::VectorXd& out, int rows, std::string vector_name);
};

template <typename T>
MultiChannelKalmanFilter<T>::MultiChannelKalmanFilter() : nh_("~"), params_{std::string(nh_.getNamespace() + "/KalmanFilter")} {}

template <typename T>
bool MultiChannelKalmanFilter<T>::fromStdVectorToEigenMatrix(std::vector<double>& in, Eigen::MatrixXd& out, int rows, int columns, std::string matrix_name) {
  if (in.size() != rows * columns) { ROS_ERROR("%s is not valid!", matrix_name.c_str()); return false; }
  out.resize(rows, columns);
  std::vector<double>::iterator it = in.begin();
  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < columns; ++j) 
    {
      out(i, j) = *it;
      ++it;
    }
  }
}

template <typename T>
bool MultiChannelKalmanFilter<T>::fromStdVectorToEigenVector(std::vector<double>& in, Eigen::VectorXd& out, int rows, std::string vector_name) {
  if (in.size() != rows) { ROS_ERROR("%s vector is not valid!", vector_name.c_str()); return false; }
  out.resize(rows);
  for (int i = 0; i < rows; ++i)
  {
    out(i) = in[i];
  }
}

template <typename T>
bool MultiChannelKalmanFilter<T>::configure() {
  
  params_.fromParamServer();
  
  dt = params_.dt;
  n = params_.n;
  m = params_.m;
  
  I = Eigen::MatrixXd::Zero(n, n);
  I.setIdentity();
  std::vector<double> temp;
  
  temp = params_.A;
  if (!fromStdVectorToEigenMatrix(temp, A, n, n, "State matrix")) { return false; }
  
  temp = params_.C;
  if (!fromStdVectorToEigenMatrix(temp, C, m, n, "Output matrix")) { return false; }
  
  temp = params_.Q;
  if (!fromStdVectorToEigenMatrix(temp, Q, n, n, "Process noise covariance")) { return false; }
  
  temp = params_.R;
  if (!fromStdVectorToEigenMatrix(temp, R, m, m, "Measurement noise covariance")) { return false; }
  
  temp = params_.P;
  if (!fromStdVectorToEigenMatrix(temp, P0, n, n, "Estimate error covariance")) { return false; }
  
  temp = params_.x0;
  if (!fromStdVectorToEigenVector(temp, x_hat, n, "Start state vector")) { return false; }
  
  x_hat_new = Eigen::VectorXd::Zero(n);
  P = P0;
  
  initialized = true;
  
  return true;
}

template <typename T>
MultiChannelKalmanFilter<T>::~MultiChannelKalmanFilter()
{}

template <typename T>
bool MultiChannelKalmanFilter<T>::update(const std::vector<T>& data_in, std::vector<T>& data_out)
{
  if(!initialized) { ROS_ERROR("Kalman: Filter is not initialized!"); return false; }
  
  if(data_in.size() != m) { ROS_ERROR("Kalman: Not valid measurement vector!"); return false; }
  
  Eigen::VectorXd y(m);
  for (int i = 0; i < m; ++i) { 
    y(i) = data_in[i];	
  }
  
  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse(); // nxm
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P; 
  x_hat = x_hat_new;
  
  data_out.resize(m);
  for (int i = 0; i < m; ++i) {
    data_out[i] = x_hat(i);
  }
  
  return true;
};

}
#endif

