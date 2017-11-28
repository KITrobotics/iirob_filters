
#ifndef IIROB_FILTERS_FILTER_BASE_H
#define IIROB_FILTERS_FILTER_BASE_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <filters/filter_base.h>
#include <pluginlib/class_loader.h>
#include <iostream>
#include <typeinfo>  

namespace iirob_filters{
    
template <typename T>
class IIrobFilterBase : public filters::FilterBase<T>
{
public:
  IIrobFilterBase(): ns_("")
  {
    data_type_ = "geometry_msgs::WrenchStamped";
    //TODO
    //std::cout<<"id: "<<typeid(T).name()<<std::endl;
  };
  
  /** \brief Default constructor used by Filter Factories
   */
  bool configure(const std::string& param_name, ros::NodeHandle node_handle)
  {    
    ns_ = node_handle.getNamespace()+"/"+param_name;   
    std::cout<<"ns:"<<ns_<<std::endl;
    pluginlib::ClassLoader<filters::FilterBase<T> > loader("filters", std::string("filters::FilterBase<") + data_type_ + std::string(">"));
    std::string lib_string = "";
    std::vector<std::string> libs = loader.getDeclaredClasses();
    for (unsigned int i = 0 ; i < libs.size(); i ++)
    {
      lib_string = lib_string + std::string(", ") + libs[i];
    }    
    ROS_DEBUG("In IIrobFilterBase ClassLoader found the following libs: %s", lib_string.c_str());
    
    XmlRpc::XmlRpcValue config;
    if (!node_handle.getParam(param_name, config))
    {
      ROS_ERROR("Could not find parameter %s on the server, are you sure that it was pushed up correctly?", param_name.c_str());
      return false;
    }   
	bool found = false;
    std::cout<<"config t: "<<std::string(config["type"])<<std::endl;
	for (std::vector<std::string>::iterator it = libs.begin(); it != libs.end(); ++it)
	{        
	    if (*it == std::string(config["type"]))
	      {
		found = true;
		break;
	      }
	}
	if (!found)
    {
	    ROS_ERROR("Couldn't find filter of type %s", std::string(config["type"]).c_str());
	    return false;
	}
      
    bool result = true;    
    boost::shared_ptr<filters::FilterBase<T> > p(loader.createInstance(std::string(config["type"])));
    if (p.get() == NULL){
       std::cout<<"null"<<std::endl;
       return false;
    }    
    reference_pointers_.push_back(p);
    result = reference_pointers_[0]->configure(config);    
    std::string type = config["type"];
    std::string name = config["name"];
    ROS_INFO("%s: Configured %s:%s filter at %p\n", ns_.c_str(), type.c_str(),
                name.c_str(),  p.get());
    return result;
    
  }
  bool update(const T& data_in, T& data_out)
  {
      return reference_pointers_[0]->update(data_in,data_out);      
  }
  
  bool update(const std::vector<T>& data_in, std::vector<T>& data_out)
  {
      return reference_pointers_[0]->update(data_in,data_out);      
  }
  bool configure()
  {
      std::cout<<"configure() iirob base"<<std::endl;
  }    
protected:
  std::string ns_;
private:
  std::string data_type_;
  std::vector<boost::shared_ptr<filters::FilterBase<T> > > reference_pointers_;   ///<! A vector of pointers to currently constructed filters
};
}
#endif
