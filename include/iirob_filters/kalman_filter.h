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
#include <math.h>


namespace iirob_filters {
template <typename T>
class MultiChannelKalmanFilter : public filters::MultiChannelFilterBase<T>
{
public:
  MultiChannelKalmanFilter();
  ~MultiChannelKalmanFilter();
  virtual bool configure();
  bool configure(const std::vector<T>& init_state_vector, const std::string param_namespace = "");
  bool configure(const std::string& param_namespace);
  virtual bool update(const std::vector<T>& data_in, std::vector<T>& data_out);
  bool update(const std::vector<T>& data_in, std::vector<T>& data_out, const double& delta_t, bool update_Q_matrix = false);
  bool predict(std::vector<T>& data_out);
  bool computePrediction(std::vector<T>& data_out);
  bool isInitializated(); 
  bool getCurrentState(std::vector<T>& data_out); 
  bool getErrorCovarianceMatrix(Eigen::MatrixXd& data_out);
  bool resetErrorCovAndState();
  bool getGatingMatrix(Eigen::MatrixXd& data_out);
  bool likelihood(const std::vector<T>& data_in, double& data_out);
  
private:
  // Matrices for computation
  Eigen::MatrixXd A, At, C, Q, Q_coeff, Q_exponent, R, P, K, P0, gatingMatrix;

  // System dimensions
  int m, n;

  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;
  
  // Variance of process noise (for a time dependent Q)
  double Q_variance;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::VectorXd x_hat, x_hat_new;
  
  ros::NodeHandle nh;
  
  bool can_update_Q_matrix;
  
  bool fromStdVectorToEigenMatrix(std::vector<double>& in, Eigen::MatrixXd& out, int rows, int columns, std::string matrix_name);
  bool fromStdVectorToEigenVector(std::vector<double>& in, Eigen::VectorXd& out, int rows, std::string vector_name);
  
  double fac(double x);
  
  bool getParams(iirob_filters::KalmanFilterParameters&, const std::string&);
  
  bool isDynamicUpdate;
};

template <typename T>
double MultiChannelKalmanFilter<T>::fac(double x){
    double f;
    if (x==0 || x==1) {
      f = 1;
    }
    else{
      f = fac(x-1) * x;
    }
    return f;
}

template <typename T>
MultiChannelKalmanFilter<T>::MultiChannelKalmanFilter() : nh("~") 
{
  initialized = isDynamicUpdate = can_update_Q_matrix = false;
}

template <typename T>
bool MultiChannelKalmanFilter<T>::fromStdVectorToEigenMatrix(std::vector<double>& in, Eigen::MatrixXd& out, int rows, 
							     int columns, std::string matrix_name) {
  if (in.size() != rows * columns || in.size() == 0) { ROS_ERROR("%s is not valid!", matrix_name.c_str()); return false; }
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
bool MultiChannelKalmanFilter<T>::isInitializated()
{
    return initialized; 
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
bool MultiChannelKalmanFilter<T>::configure(const std::vector<T>& init_state_vector, const std::string param_namespace) {
  
  if (!configure(param_namespace)) { return false; }
  
  if (init_state_vector.size() != n) { ROS_ERROR("Kalman: Not valid init state vector!"); return false; }
  
  for (int i = 0; i < init_state_vector.size(); i++)
  {
    x_hat(i) = init_state_vector[i];
  }
  return true;
}

template <typename T>
bool MultiChannelKalmanFilter<T>::configure() {
  return configure("");
}

template <typename T>
bool MultiChannelKalmanFilter<T>::configure(const std::string& param_namespace) {
  std::vector<double> temp;
  std::string full_namespace = nh.getNamespace() + "/";
  if (param_namespace != "") 
  {
    full_namespace += param_namespace;
    iirob_filters::KalmanFilterParameters dynamic_params{full_namespace}; 
    dynamic_params.fromParamServer();
    if (!getParams(dynamic_params, full_namespace)) { return false; }
  }
  else 
  {
    full_namespace += "KalmanFilter";
    iirob_filters::KalmanFilterParameters default_namespace_params{full_namespace};
    default_namespace_params.fromParamServer();
    if (!getParams(default_namespace_params, full_namespace)) { return false; }
  }
  
  I = Eigen::MatrixXd::Zero(n, n);
  I.setIdentity();
  
  x_hat_new = Eigen::VectorXd::Zero(n);
  P = P0;
  
  gatingMatrix = C * P * C.transpose() + R;
  
  initialized = true;
  return true;
}

template <typename T>
bool MultiChannelKalmanFilter<T>::getParams(iirob_filters::KalmanFilterParameters& parameters, const std::string& param_namespace)
{
  if(ros::param::has(std::string(param_namespace + "/" + "dt")))
  {
    dt = parameters.dt;
  }
  else 
  { 
    ROS_ERROR("Kalman: dt is not available!");
    return false;
  }
  
  if(ros::param::has(std::string(param_namespace + "/" + "n")))
  {
    n = parameters.n;
  }
  else 
  { 
    ROS_ERROR("Kalman: n is not available!");
    return false;
  }
  
  if(ros::param::has(std::string(param_namespace + "/" + "m")))
  {
    m = parameters.m;
  }
  else 
  { 
    ROS_ERROR("Kalman: m is not available!");
    return false;
  }
  
  if (dt <= 0 || n <= 0 || m <= 0) 
  {
    ROS_ERROR("Kalman: dt, n or m is not valid! (dt <= 0 or n <= 0 or m <= 0)"); 
    return false;
  }
  
  std::vector<double> temp;
  
  if(ros::param::has(std::string(param_namespace + "/" + "A")))
  {
    temp = parameters.A;
    if (!fromStdVectorToEigenMatrix(temp, A, n, n, "State matrix")) { return false; }
  }
  else 
  { 
    ROS_ERROR("A is not available!"); 
    return false;
  }
  
  if(ros::param::has(std::string(param_namespace + "/" + "At")))
  {
    temp = parameters.At;
    if (!fromStdVectorToEigenMatrix(temp, At, n, n, "Dynamic part of state matrix")) { return false; }
    isDynamicUpdate = true;
  }
  else 
  { 
    ROS_DEBUG("At is not available!"); 
  }

  if(ros::param::has(std::string(param_namespace + "/" + "C")))
  {
    temp = parameters.C;
    if (!fromStdVectorToEigenMatrix(temp, C, m, n, "Output matrix")) { return false; }
  }
  else 
  { 
    ROS_ERROR("C is not available!"); 
    return false;
  }

  if(ros::param::has(std::string(param_namespace + "/" + "Q")))
  {
    temp = parameters.Q;
    if (!fromStdVectorToEigenMatrix(temp, Q, n, n, "Process noise covariance")) { return false; }
  }
  else 
  { 
    ROS_ERROR("Q is not available!"); 
    return false;
  }
  
  bool skipDynamicPartQ = false;
  if(isDynamicUpdate && ros::param::has(std::string(param_namespace + "/" + "Q_coeff")))
  {
    temp = parameters.Q_coeff;
    if (!fromStdVectorToEigenMatrix(temp, Q_coeff, n, n, 
      "Process noise covariance (coefficients of dynamic part of Q)")) { return false; } 
  }
  else 
  { 
    ROS_DEBUG("Q_coeff is not available!"); 
    skipDynamicPartQ = true;
  }
  
  if(isDynamicUpdate && !skipDynamicPartQ && 
    ros::param::has(std::string(param_namespace + "/" + "Q_exponent")))
  {
    temp = parameters.Q_exponent;
    if (!fromStdVectorToEigenMatrix(temp, Q_exponent, n, n, 
      "Process noise covariance (exponents of the time difference)")) { return false; } 
  }
  else 
  { 
    ROS_DEBUG("Q_exponent is not available!"); 
    skipDynamicPartQ = true;
  }
  
  if(isDynamicUpdate && !skipDynamicPartQ && 
    ros::param::has(std::string(param_namespace + "/" + "Q_variance")))
  {
    Q_variance = parameters.Q_variance;
    can_update_Q_matrix = true;
  }
  else 
  { 
    ROS_DEBUG("Q_variance is not available!"); 
  }

  if(ros::param::has(std::string(param_namespace + "/" + "R")))
  {
    temp = parameters.R;
    if (!fromStdVectorToEigenMatrix(temp, R, m, m, "Measurement noise covariance")) { return false; }
  }
  else 
  { 
    ROS_ERROR("R is not available!"); 
    return false;
  }
  

  if(ros::param::has(std::string(param_namespace + "/" + "P")))
  {
    temp = parameters.P;
    if (!fromStdVectorToEigenMatrix(temp, P0, n, n, "Estimate error covariance")) { return false; }
  }
  else 
  { 
    ROS_ERROR("P is not available!"); 
    return false;
  }
  
  if(ros::param::has(std::string(param_namespace + "/" + "x0")))
  {
    temp = parameters.x0;
    if (!fromStdVectorToEigenVector(temp, x_hat, n, "Start state vector")) { return false; }
    isDynamicUpdate = true;
  }
  else 
  { 
    ROS_DEBUG("x0 is not available!"); 
    x_hat = Eigen::VectorXd::Zero(n);
  }
}

template <typename T>
MultiChannelKalmanFilter<T>::~MultiChannelKalmanFilter()
{}

template <typename T>
bool MultiChannelKalmanFilter<T>::computePrediction(std::vector<T>& data_out)
{
  if(!initialized) { ROS_ERROR("Kalman: Filter is not initialized!"); return false; }
  
  Eigen::VectorXd v = Eigen::VectorXd::Zero(n);
  v = A * x_hat;
  
  data_out.resize(n);
  for (int i = 0; i < data_out.size(); ++i) {
    data_out[i] = v(i);
  }
  
  return true;
}

template <typename T>
bool MultiChannelKalmanFilter<T>::predict(std::vector<T>& data_out)
{
  if(!initialized) { ROS_ERROR("Kalman: Filter is not initialized!"); return false; }
  
  x_hat = A * x_hat;
  P = A*P*A.transpose() + Q;
  
  data_out.resize(n);
  for (int i = 0; i < data_out.size(); ++i) {
    data_out[i] = x_hat(i);
  }
  
  return true;
}

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
  gatingMatrix = C * P * C.transpose() + R; 
  K = P*C.transpose()*gatingMatrix.inverse(); // nxm
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P; 
  x_hat = x_hat_new;
  
  data_out.resize(n);
  for (int i = 0; i < data_out.size(); ++i) {
    data_out[i] = x_hat(i);
  }
  
  return true;
}

template <typename T>
bool MultiChannelKalmanFilter<T>::getGatingMatrix(Eigen::MatrixXd& data_out)
{
  if(!initialized) { ROS_ERROR("Kalman: Filter is not initialized!"); return false; }
  data_out = gatingMatrix;
  return true;
}

template <typename T>
bool MultiChannelKalmanFilter<T>::getErrorCovarianceMatrix(Eigen::MatrixXd& data_out)
{
  if(!initialized) { ROS_ERROR("Kalman: Filter is not initialized!"); return false; }
  data_out = P;
  return true;
}

template <typename T>
bool MultiChannelKalmanFilter<T>::update(const std::vector<T>& data_in, std::vector<T>& data_out, 
					 const double& delta_t, bool update_Q_matrix)
{
  if(!initialized) { ROS_ERROR("Kalman: Filter is not initialized!"); return false; }
  if(!isDynamicUpdate) { ROS_ERROR("Kalman: Dynamic update is not available!"); return false; }
  if(data_in.size() != m) { ROS_ERROR("Kalman: Not valid measurement vector!"); return false; }

  Eigen::VectorXd y(m);
  for (int i = 0; i < m; ++i) { 
    y(i) = data_in[i];	
  }
  
  Eigen::MatrixXd A_t = At;
  
  for(int i = 0; i < n; i++){
    for(int j = 0; j < n; j++){
      if( A_t(i,j) ){
           A_t(i,j) = pow( delta_t , A_t(i,j) )/fac( A_t(i,j) );
      }
    }
  }

  //update Q matrix if requested
  Eigen::MatrixXd Q_updated = Q;
  if (can_update_Q_matrix && update_Q_matrix)
  {
    for(int i = 0; i < n; i++){
        for(int j = 0; j < n; j++){
            // adds x*delta_t^y to entry in Q 
            Q_updated(i,j) = Q(i,j) + Q_variance * Q_coeff(i,j)*pow(delta_t, Q_exponent(i,j));
        }
    }
  }
  
  
  x_hat_new = (A_t+A) * x_hat;
  P = (A_t+A)*P*((A_t+A).transpose()) + Q_updated;
  gatingMatrix = C * P * C.transpose() + R; 
  K = P*C.transpose()*gatingMatrix.inverse(); // nxm
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P; 
  x_hat = x_hat_new;
  
  data_out.resize(n);
  for (int i = 0; i < n; ++i) {
    data_out[i] = x_hat(i);
  }
  return true;
}

template <typename T>
bool MultiChannelKalmanFilter<T>::getCurrentState(std::vector<T>& data_out)
{
  if(!initialized) { ROS_ERROR("Kalman: Filter is not initialized!"); return false; }
  data_out.resize(n);
  for (int i = 0; i < n; ++i) {
      data_out[i] = x_hat(i);
  }
  return true;    
}

template <typename T>
bool MultiChannelKalmanFilter<T>::likelihood(const std::vector<T>& data_in, double& data_out)
{
  if(!initialized) { ROS_ERROR("Kalman: Filter is not initialized!"); return false; }
  
  if(data_in.size() != m) { ROS_ERROR("Kalman: Not valid measurement vector!"); return false; }
  
  Eigen::VectorXd measurement(m);
  for (int i = 0; i < m; ++i) { 
    measurement(i) = data_in[i];	
  }
  
  // convert prediction to measurement space
  Eigen::VectorXd prediction = C * A * x_hat;
  
  // vector of prediction (origin = current state)
  Eigen::VectorXd continuousPrediction = prediction - C * x_hat;

  // assumed interpolated prediction = current state + dt * prediction
  Eigen::VectorXd timeShiftedPrediction = ( C * x_hat ) + ( dt * continuousPrediction );
  
  Eigen::VectorXd d = timeShiftedPrediction - measurement;
  
  // calculate exponent
  const double e = -0.5 * d.transpose() * gatingMatrix.inverse() * d;

  // get normal distribution value
  data_out = (1. / (std::pow(2. * M_PI, (double) m * .5)
		  * std::sqrt(gatingMatrix.determinant()))) * std::exp(e);
  
  return true;
}

template <typename T>
bool MultiChannelKalmanFilter<T>::resetErrorCovAndState()
{
  if(!initialized) { ROS_ERROR("Kalman: Filter is not initialized!"); return false; }
  
  for (int i = m; i < n; i++) 
  {
    x_hat(i) = 0.0;
  }
  P = P0;
  return true;
}


}
#endif
