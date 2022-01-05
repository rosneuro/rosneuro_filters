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

		bool configure(const std::string& param_name, ros::NodeHandle nh = ros::NodeHandle());
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
};

}

#include "../src/Filter.cpp"

#endif
