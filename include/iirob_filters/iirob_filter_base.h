
#ifndef IIROB_FILTERS_FILTER_BASE_H
#define IIROB_FILTERS_FILTER_BASE_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <filters/filter_base.h>

namespace iirob_filters{
    
template <typename T>
class iirobFilterBase : public filters::FilterBase<T>
{
public:
  /** \brief Default constructor used by Filter Factories
   */
  bool configure(const std::string& param_name, ros::NodeHandle node_handle)
  {    
    XmlRpc::XmlRpcValue config;
    if (!node_handle.getParam(param_name, config))
    {
      ROS_ERROR("Could not find parameter %s on the server, are you sure that it was pushed up correctly?", param_name.c_str());
      return false;
    }
    std::string ns = node_handle.getNamespace()+"/"+param_name;
    return configure(config,ns);
    
  }
   bool configure(XmlRpc::XmlRpcValue& config, const std::string& ns)
  {
    if (filters::FilterBase<T>::configured_)
    {
      ROS_WARN("Filter %s of type %s already being reconfigured", filters::FilterBase<T>::filter_name_.c_str(), filters::FilterBase<T>::filter_type_.c_str());
    };
    filters::FilterBase<T>::configured_ = false;
    bool retval = true;

    retval = retval && filters::FilterBase<T>::loadConfiguration(config);
    retval = retval && configure(ns);
    filters::FilterBase<T>::configured_ = retval;
    return retval;
  }
protected:
  virtual bool configure(std::string ns)=0;
};
}
#endif
