#include <ros/ros.h>
#include "rosneuro_filters/FilterChainNode.h"


int main(int argc, char** argv) {
	ros::init(argc, argv, "filter_chain_node");

	rosneuro::FilterChainNode filter_chain;

	if(!filter_chain.configure()) {
		ROS_ERROR("[filter chain_node] Configuration failed");
		return -1;
	}

    filter_chain.run();
	ros::shutdown();
	return 0;
}
