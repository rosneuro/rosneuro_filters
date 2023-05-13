#ifndef ROSNEURO_FILTERS_FCNODE_H_
#define ROSNEURO_FILTERS_FCNODE_H_

#include <ros/ros.h>
#include <rosneuro_msgs/NeuroFrame.h>
#include "rosneuro_filters/FilterChain.h"

namespace rosneuro {

class FilterChainNode {

	public:
		FilterChainNode(void);
		~FilterChainNode(void);


		bool configure(void);
		void run(void);

	private:
		void on_received_neurodata(const rosneuro_msgs::NeuroFrame& msg);

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle p_nh_;
		ros::Subscriber sub_;
		ros::Publisher  pub_;
		std::string 	cfg_name_;
		FilterChain<float> chain_;
		rosneuro_msgs::NeuroFrame neuromsg_;
};


}

#endif
