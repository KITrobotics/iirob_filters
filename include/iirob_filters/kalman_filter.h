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
  bool configure(const std::string& param_namespace);
  virtual bool update(const std::vector<T>& data_in, std::vector<T>& data_out);
  bool update(const std::vector<T>& data_in, std::vector<T>& data_out, const double& delta_t);

private:
    // Matrices for computation
  Eigen::MatrixXd A, At, C, Q, R, P, K, P0;

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
  int  fac(int x);
  
};

template <typename T>
int MultiChannelKalmanFilter<T>::fac(int x){
    int f;
    if(x==0||x==1){
      f=1;
    }
    else{
      f=fac(x-1)*x;
    }
    return f;
}
  
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
  if (!fromStdVectorToEigenMatrix(temp, A, n, n, "Static part of state matrix")) { return false; }
  
  temp = params_.At;
  if (!fromStdVectorToEigenMatrix(temp, At, n, n, "Dynamic part of state matrix")) { return false; }
  
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
bool MultiChannelKalmanFilter<T>::configure(const std::string& param_namespace) {
  params_.fromParamServer();
  
  iirob_filters::KalmanFilterParameters* temp_params_p;
  if (param_namespace != "") 
  { 
    iirob_filters::KalmanFilterParameters temp_params{std::string(nh_.getNamespace() + "/" + param_namespace)}; 
    temp_params.fromParamServer();
    temp_params_p = &temp_params;
  }
    
  if (param_namespace != "") 
  {
    dt = temp_params_p->dt;
    n = temp_params_p->n;
    m = temp_params_p->m;
  }
  else 
  {
    dt = params_.dt;
    n = params_.n;
    m = params_.m;
  }
  
  I = Eigen::MatrixXd::Zero(n, n);
  I.setIdentity();
  
  std::vector<double> temp;
  
   if (param_namespace != "") 
      temp = temp_params_p->A;
    else 
      temp = params_.A;
  if (!fromStdVectorToEigenMatrix(temp, A, n, n, "Static part of state matrix")) { return false; }
  
  if (param_namespace != "") 
      temp = temp_params_p->At;
    else 
      temp = params_.At;
  if (!fromStdVectorToEigenMatrix(temp, At, n, n, "Dynamic part of state matrix")) { return false; }
    
    if (param_namespace != "") 
      temp = temp_params_p->C;
    else
      temp = params_.C;
  if (!fromStdVectorToEigenMatrix(temp, C, m, n, "Output matrix")) { return false; }
  
    if (param_namespace != "") 
      temp = temp_params_p->Q;
    else
      temp = params_.Q;
  if (!fromStdVectorToEigenMatrix(temp, Q, n, n, "Process noise covariance")) { return false; }
  
    if (param_namespace != "") 
      temp = temp_params_p->R;
    else
      temp = params_.R;
  if (!fromStdVectorToEigenMatrix(temp, R, m, m, "Measurement noise covariance")) { return false; }
  
    if (param_namespace != "") 
      temp = temp_params_p->P;
    else
      temp = params_.P;
  if (!fromStdVectorToEigenMatrix(temp, P0, n, n, "Estimate error covariance")) { return false; }
  
    if (param_namespace != "") 
      temp = temp_params_p->x0;
    else
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
  
  data_out.resize(n);
  for (int i = 0; i < n; ++i) {
    data_out[i] = x_hat(i);
  }
  
  return true;
}

template <typename T>
bool MultiChannelKalmanFilter<T>::update(const std::vector<T>& data_in, std::vector<T>& data_out, const double& delta_t)
{
  if(!initialized) { ROS_ERROR("Kalman: Filter is not initialized!"); return false; }
  
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
  x_hat_new = (A_t+A) * x_hat;
  P = (A_t+A)*P*((A_t+A).transpose()) + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse(); // nxm
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P; 
  x_hat = x_hat_new;
  
  data_out.resize(n);
  for (int i = 0; i < n; ++i) {
    data_out[i] = x_hat(i);
  }
  return true;
};

}
#endif
