#ifndef ROSNEURO_FILTERS_FILTER_HPP
#define ROSNEURO_FILTERS_FILTER_HPP

#include <string>
#include <map>
#include <ros/ros.h>
#include "rosneuro_data/NeuroData.hpp"

namespace rosneuro {

template <typename T>
class Filter {
	public:
		Filter(void) : configured_(false) {};
		virtual ~Filter(void) {};

		bool configure(const std::string& param_name);
		bool configure(XmlRpc::XmlRpcValue& config);
		std::string type(void) const;
		std::string name(void) const;

		virtual bool apply(const NeuroData<T>& data_in, NeuroData<T>& data_out) = 0;

		bool getParam(const std::string& name, std::string& value) const;
		bool getParam(const std::string& name, bool& value) const;
		bool getParam(const std::string& name, double& value) const;
		bool getParam(const std::string& name, int& value) const;
		bool getParam(const std::string& name, unsigned  int& value) const;
		bool getParam(const std::string& name, std::vector<double>& value) const;
		bool getParam(const std::string& name, std::vector<std::string>& value) const;
		bool getParam(const std::string& name, XmlRpc::XmlRpcValue& value) const;

	protected:
		virtual bool configure(void) = 0;
		bool loadConfiguration(XmlRpc::XmlRpcValue& config);

	protected:
		bool configured_;
		std::string type_;
		std::string name_;
		std::map<std::string, XmlRpc::XmlRpcValue> params_;

	private:
		bool setNameAndType(XmlRpc::XmlRpcValue& config);
		ros::NodeHandle nh_;
};


template<typename T>
std::string Filter<T>::type(void) const {
	return this->type_;
}

template<typename T>
std::string Filter<T>::name(void) const {
	return this->name_;
}

template<typename T>
bool Filter<T>::setNameAndType(XmlRpc::XmlRpcValue& config) {

	if(config.hasMember("name") == false) {
		ROS_ERROR("Filter didn't have name defined, this is required");
		return false;
	}
	
	if(config.hasMember("type") == false) {
		ROS_ERROR("Filter didn't have type defined, this is required");
		return false;
	}

	this->name_ = std::string(config["name"]);
	this->type_ = std::string(config["type"]);
	ROS_DEBUG("Configuring Filter of type: %s with name %s", this->type_.c_str(), this->name_.c_str());
    
	return true;
}

template<typename T>
bool Filter<T>::configure(const std::string& param_name) {
	
	bool retval = false;
	XmlRpc::XmlRpcValue config;
	if (!this->nh_.getParam(param_name, config)) {
  		ROS_ERROR("Could not find parameter %s on the server, are you sure that it was pushed up correctly?", param_name.c_str());
		return false;
	}

	retval = this->configure(config);
	ROS_INFO("qui2");
	return retval;
}

template<typename T>
bool Filter<T>::configure(XmlRpc::XmlRpcValue& config) {
	if (configured_) {
		ROS_ERROR("Filter %s of type %s already being reconfigured", this->name_.c_str(), this->type_.c_str());
	}
	this->configured_ = false;
	bool retval = true;
	
	retval = retval && this->loadConfiguration(config);
	retval = retval && this->configure();
	configured_ = retval;
	ROS_INFO("qui1");
	return retval;
}

template<typename T>
bool Filter<T>::loadConfiguration(XmlRpc::XmlRpcValue& config) {
	
	if(config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
		ROS_ERROR("A filter configuration must be a map with fields name, type, and params");
		return false;
	} 
  
	if (!setNameAndType(config)) {
		return false;
	}

	//check to see if we have parameters in our list
	if(config.hasMember("params")) {
  		
		//get the params map
  		XmlRpc::XmlRpcValue params = config["params"];

		if(params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    		ROS_ERROR("params must be a map");
			return false;
		} else {
    	
			//Load params into map
    		for(XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it) {
      			ROS_DEBUG("Loading param %s", it->first.c_str());
      			this->params_[it->first] = it->second;
			}
		}
	}

	return true;
}

template<typename T>
bool Filter<T>::getParam(const std::string& name, std::string& value) const {
    
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeString)
		return false;

    auto tmp = it->second;
    value = std::string(tmp);
    return true;
}

template<typename T>
bool Filter<T>::getParam(const std::string& name, bool& value) const {
	
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;

	if(it->second.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
		return false;

	auto tmp = it->second;
	value = (bool)(tmp);
	return true;
}

template<typename T>
bool Filter<T>::getParam(const std::string& name, double& value) const {
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;
	
	if(it->second.getType() != XmlRpc::XmlRpcValue::TypeDouble && it->second.getType() != XmlRpc::XmlRpcValue::TypeInt)
		return false;
	
	auto tmp = it->second;
	value = it->second.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(tmp) : (double)(tmp);
	return true;
}

template<typename T>
bool Filter<T>::getParam(const std::string& name, int& value) const {
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;
	
	if(it->second.getType() != XmlRpc::XmlRpcValue::TypeInt)
		return false;
	
	auto tmp = it->second;
	value = tmp;
	return true;
}

template<typename T>
bool Filter<T>::getParam(const std::string& name, unsigned int& value) const {
	int signed_value;
	if (!this->getParam(name, signed_value))
		return false;
	if (signed_value < 0)
		return false;
	value = signed_value;
	return true;
}

template<typename T>
bool Filter<T>::getParam(const std::string& name, std::vector<double>& value) const {
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;
	
	value.clear();
	
	if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
		return false;
	
	XmlRpc::XmlRpcValue double_array = it->second;
	
	for (auto i = 0; i < double_array.size(); ++i){
		if(double_array[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && double_array[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
	    return false;
		}
		
		double double_value = double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(double_array[i]) : (double)(double_array[i]);
		value.push_back(double_value);
	}
	
	return true;
}

template<typename T>
bool Filter<T>::getParam(const std::string& name, std::vector<std::string>& value) const {
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;
	
	value.clear();
	
	if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
		return false;
	
	XmlRpc::XmlRpcValue string_array = it->second;
	
	for (auto i = 0; i < string_array.size(); ++i) {
		if(string_array[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
	    	return false;
	  	}
	
	  	value.push_back(string_array[i]);
	}
	
	return true;
}

template<typename T>
bool Filter<T>::getParam(const std::string& name, XmlRpc::XmlRpcValue& value) const {
	auto it = this->params_.find(name);
	if (it == this->params_.end())
		return false;
	
	auto tmp = it->second;
	value = tmp;
	return true;
}

}

#endif
