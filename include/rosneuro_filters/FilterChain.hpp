#ifndef ROSNEURO_FILTERS_FILTERCHAIN_HPP
#define ROSNEURO_FILTERS_FILTERCHAIN_HPP

#include <vector>
#include <memory>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "rosneuro_filters/Filter.hpp"

namespace rosneuro {

template<typename T>
class FilterChain {

	public:
		FilterChain(const std::string data_type);
		~FilterChain(void);
		
		bool configure(const std::string param_name, ros::NodeHandle nh = ros::NodeHandle());
		bool configure(XmlRpc::XmlRpcValue& config, const std::string& proc_ns);
		bool apply(const NeuroData<T>& data_in, NeuroData<T>& data_out);
		bool clear(void);

	private:
		bool configured_;
		std::vector<boost::shared_ptr<rosneuro::Filter<T>>> procs_;
		pluginlib::ClassLoader<rosneuro::Filter<T> > loader_;

};


template<typename T>
FilterChain<T>::FilterChain(const std::string data_type) : 
		loader_("rosneuro_filters", std::string("rosneuro::Filter<") + data_type + std::string(">")), 
		configured_(false) {
	
	std::string lib_string = "";
	std::vector<std::string> libs = this->loader_.getDeclaredClasses();

	for (auto i = 0 ; i < libs.size(); i++) {
       lib_string = lib_string + std::string(", ") + libs[i];
	}    
     
	ROS_DEBUG("In FilterChain ClassLoader found the following libs: %s", lib_string.c_str());
}

template<typename T>
FilterChain<T>::~FilterChain(void) {
	this->clear();
}

template<typename T>
bool FilterChain<T>::clear(void) {
	
	this->configured_ = false;
	this->procs_.clear();
	return true;
}

template<typename T>
bool FilterChain<T>::apply(const NeuroData<T>& data_in, NeuroData<T>& data_out) {

	unsigned int nprocs = this->procs_.size();

	for(auto it=this->procs_.begin(); it != this->procs_.end(); ++it) {
		ROS_INFO("Applying filter '%s'", (*it)->name());
	}

	return true;

}


template<typename T>
bool FilterChain<T>::configure(const std::string param_name, ros::NodeHandle nh) {

	XmlRpc::XmlRpcValue config;
	if(nh.getParam(param_name + "/filters_chain", config)) {
		std::string resolved_name = nh.resolveName(param_name).c_str();
	} else if(!nh.getParam(param_name, config)) {
		ROS_DEBUG("Could not load the filter chain configuration from parameter %s, are you sure it was pushed to the parameter server? Assuming that you meant to leave it empty.", param_name.c_str());
		configured_ = true;
		return true;
	}

	return this->configure(config, nh.getNamespace());
}

template<typename T>
bool FilterChain<T>::configure(XmlRpc::XmlRpcValue& config, const std::string& filt_ns) {

	//Verify proper naming and structure    
	if (config.getType() != XmlRpc::XmlRpcValue::TypeArray) {
  		ROS_ERROR("%s: The filter chain specification must be a list. but is of of XmlRpcType %d", 
				filt_ns.c_str(), config.getType());
  		ROS_ERROR("The xml passed in is formatted as follows:\n %s", config.toXml().c_str());
		return false;
	}

	//Iterate over all filter in filters (may be just one)
	for (auto i = 0; i < config.size(); ++i) {
  		if(config[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    		ROS_ERROR("%s: Filters must be specified as maps, but they are XmlRpcType:%d", 
					  filt_ns.c_str(), config[i].getType());
    		return false;
		} else if (!config[i].hasMember("type")) {
    		ROS_ERROR("%s: Could not add a filter because no type was given", filt_ns.c_str());
    		return false;
  		} else if (!config[i].hasMember("name")) {
			ROS_ERROR("%s: Could not add a filter because no name was given", filt_ns.c_str());
    		return false;
		} else {
    		//Check for name collisions within the list itself.
    		for (auto j = i + 1; j < config.size(); ++j) {
      			if(config[j].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        			ROS_ERROR("%s: Filters must be specified as maps, but they are XmlRpcType:%d", 
							  filt_ns.c_str(), config[j].getType());
        			return false;
      			}

      			if(!config[j].hasMember("name") 
				   ||config[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString
				   ||config[j]["name"].getType() != XmlRpc::XmlRpcValue::TypeString) {
        				ROS_ERROR("%s: Filters names must be strings, but they are XmlRpcTypes:%d and %d", 
								  filt_ns.c_str(), config[i].getType(), config[j].getType());
        				return false;
      			}

      			std::string namei = config[i]["name"];
      			std::string namej = config[j]["name"];
      			
				if (namei == namej) {
        			ROS_ERROR("%s: A self_process with the name %s already exists", 
							  filt_ns.c_str(), namei.c_str());
        			return false;
      			}
    		}


			if (std::string(config[i]["type"]).find("/") == std::string::npos) {
        		ROS_ERROR("Bad filter type %s. Filter type must be of form <package_name>/<process_name>", 
						  std::string(config[i]["type"]).c_str());
    			return false;
      		}
    
			//Make sure the filter chain has a valid type
    		std::vector<std::string> libs = this->loader_.getDeclaredClasses();
    		bool found = false;
    		for (auto it = libs.begin(); it != libs.end(); ++it) {
        		if (*it == std::string(config[i]["type"])) {
            		found = true;
            		break;
          		}
      		}
    		
			if (!found) {
        		ROS_ERROR("Couldn't find filter of type %s", std::string(config[i]["type"]).c_str());
        		return false;
      		}
    
  		}
	}

	bool result = true;    

   	for (int i = 0; i < config.size(); ++i) {
  
		boost::shared_ptr<rosneuro::Filter<T> > p(this->loader_.createInstance(config[i]["type"]));

  		if (p.get() == nullptr)
    		return false;
  		
		result = result && p.get()->configure(config[i]);    
  		this->procs_.push_back(p);
  		
		std::string type = config[i]["type"];
  		std::string name = config[i]["name"];
  		ROS_DEBUG("%s: Configured %s:%s filter at %p\n", filt_ns.c_str(), 
				type.c_str(), name.c_str(),  p.get());
	}

	if (result == true) {
  		this->configured_ = true;
	}
	
	return result;
}

}


#endif
