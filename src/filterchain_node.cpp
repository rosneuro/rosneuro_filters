#include <ros/ros.h>
#include "rosneuro_filters/FilterChainNode.h"


int main(int argc, char** argv) {

	ros::init(argc, argv, "filterchain_node");

	rosneuro::FilterChainNode filterchain;

	if(filterchain.configure() == false) {
		ROS_ERROR("[filterchain_node] Configuration failed");
		return -1;
	}

	filterchain.run();

	ros::shutdown();


	return 0;
}
