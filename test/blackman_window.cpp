#include <ros/ros.h>
#include "rosneuro_filters/rosneuro_filters_utilities.hpp"
#include "rosneuro_filters/Blackman.hpp"

int main(int argc, char** argv) {

	ros::init(argc, argv, "blackman_window");
	
	std::string datapath;
	int framesize;
	
	rosneuro::Filter<double>* blackman = new rosneuro::Blackman<double>();

	if(ros::param::get("~datapath", datapath) == false) {
		ROS_ERROR("Cannot find 'datapath' parameter");
		return 0;
	}
	
	if(ros::param::get("~framesize", framesize) == false) {
		ROS_ERROR("Cannot find 'framesize' parameter");
		return 0;
	}

	const std::string fileinput = datapath + "/test/rawdata.csv";
	const std::string fileout   = datapath + "/test/blackman_window.csv";
	
	// Load input data
	rosneuro::DynamicMatrix<double> input = readCSV<double>(fileinput);
	
	// Allocate time variables
	ros::WallTime start_blackman, stop_blackman;

	// Apply the filter
	ROS_INFO("Applying filter");
	start_blackman = ros::WallTime::now();
	rosneuro::DynamicMatrix<double> output = blackman->apply(input);
	stop_blackman = ros::WallTime::now();

	ROS_INFO("Blackman applied on data in %9.6f ms", ((stop_blackman-start_blackman).toNSec())/1000.0f);

	// Writing the filtered data
	writeCSV<double>(fileout, output);

	ros::shutdown();
	

	return 0;

}
