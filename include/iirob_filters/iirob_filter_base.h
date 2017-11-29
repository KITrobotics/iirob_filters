
#ifndef IIROB_FILTERS_FILTER_BASE_H
#define IIROB_FILTERS_FILTER_BASE_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <filters/filter_base.h>
#include <pluginlib/class_loader.h>
#include <iostream>
#include <typeinfo> 
#include <cxxabi.h>

namespace iirob_filters{
    
template <typename T>
class IIrobFilterBase : public filters::FilterBase<T>
{
public:
  IIrobFilterBase() {};
  
  bool configure(XmlRpc::XmlRpcValue& config, std::string ns)
  {        
    ns_ = ns;
    return filters::FilterBase<T>::configure(config);   
  }
protected:
  std::string ns_;    
};
}
#endif
