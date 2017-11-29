
#ifndef IIROB_FILTERS_FILTER_CHAIN_H
#define IIROB_FILTERS_FILTER_CHAIN_H

#include <ros/ros.h>
#include <boost/pointer_cast.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <filters/filter_base.h>
#include <filters/filter_chain.h>
#include <iirob_filters/iirob_filter_base.h>
#include <pluginlib/class_loader.h>
#include <iostream>
#include <typeinfo> 
#include <cxxabi.h>

namespace iirob_filters{
template <typename T>
class IIrobFilterChain
{
private:
     pluginlib::ClassLoader<filters::FilterBase<T> >* loader_;
protected:
    ///<! A vector of pointers to currently constructed filters
    std::vector<boost::shared_ptr<IIrobFilterBase<T>> > reference_pointers_iirob_;
    std::vector<boost::shared_ptr<filters::FilterBase<T>> > reference_pointers_;
public:
  IIrobFilterChain()
  {
    int status;
    std::string type_name = abi::__cxa_demangle(typeid(T).name(), 0, 0, &status);
    int position = type_name.find('_<');    
    data_type_=type_name.substr(0,position-1); 
    loader_ = new pluginlib::ClassLoader<filters::FilterBase<T>>("filters", std::string("filters::FilterBase<") + data_type_ + std::string(">")); 
  }
  bool configure(std::string param_name, ros::NodeHandle node = ros::NodeHandle())
  {
    XmlRpc::XmlRpcValue config;
    if (!node.getParam(param_name, config))
    {
        ROS_ERROR("Could not find parameter %s on the server, are you sure that it was pushed up correctly?", param_name.c_str());
        return false;
    }
    return this->configure(config, node.getNamespace()+"/"+param_name);
  }
  
  bool configure(XmlRpc::XmlRpcValue& config, const std::string& filter_ns)
  {       
      ns_ = filter_ns;
      std::string lib_string = "";
      std::vector<std::string> libs = loader_->getDeclaredClasses();
      for (unsigned int i = 0 ; i < libs.size(); i ++)
      {
        lib_string = lib_string + std::string(", ") + libs[i];
      }    
      ROS_DEBUG("In IIrobFilterBase ClassLoader found the following libs: %s", lib_string.c_str());
      
      
      bool found = false;            
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
      std::string type = config["type"];
      int position = type.find('/');    
      std::string lib_name = type.substr(0,position);	  
      
      bool result = true; 
      boost::shared_ptr<filters::FilterBase<T> > p(loader_->createInstance(std::string(config["type"])));
      if (p.get() == NULL){
       std::cout<<"Null"<<std::endl;
       return false;
      }

      if(lib_name=="iirob_filters")
      {
        lib_name_ =lib_name;
        reference_pointers_iirob_.push_back(boost::static_pointer_cast<IIrobFilterBase<T>>(p));      
        result = reference_pointers_iirob_[0]->configure(config,ns_); 
      }
      else
      { 
        lib_name_ =lib_name;
        reference_pointers_.push_back(p);        
        result = reference_pointers_[0]->configure(config); 
      }
                         
      //std::string type = config["type"];
      std::string name = config["name"];
      ROS_DEBUG("%s: Configured %s:%s filter at %p\n", ns_.c_str(), type.c_str(),
                name.c_str(),  p.get());
      return result;
  }    
  bool update(const T& data_in, T& data_out)
  {
      if(lib_name_=="iirob_filters")
        return reference_pointers_iirob_[0]->update(data_in,data_out);      
      else
        return reference_pointers_[0]->update(data_in,data_out);      
  }
  
  bool update(const std::vector<T>& data_in, std::vector<T>& data_out)
  {
     if(lib_name_=="iirob_filters")
        return reference_pointers_iirob_[0]->update(data_in,data_out);      
     else
        return reference_pointers_[0]->update(data_in,data_out);        
  }
  bool configure()
  {
      ROS_DEBUG("You are in IIrobFilterBase configure();");
      return true;  
  }    
protected:
  std::string ns_;
  std::string lib_name_;

private:
  std::string data_type_;
};
}
#endif
