#include <ros/ros.h>
#include "rosneuro_filters/rosneuro_filters_utilities.hpp"
#include "rosneuro_filters/Flattop.hpp"

int main(int argc, char** argv) {

	ros::init(argc, argv, "flattop_window");
	
	std::string datapath;
	
	rosneuro::Filter<double>* flattop = new rosneuro::Flattop<double>();

	if(ros::param::get("~datapath", datapath) == false) {
		ROS_ERROR("Cannot find 'datapath' parameter");
		return 0;
	}
	
	const std::string fileinput = datapath + "/test/rawdata.csv";
	const std::string fileout   = datapath + "/test/flattop_window.csv";
	
	// Load input data
	rosneuro::DynamicMatrix<double> input = readCSV<double>(fileinput);
	
	// Allocate time variables
	ros::WallTime start_flattop, stop_flattop;

	// Apply the filter
	ROS_INFO("Applying filter");
	start_flattop = ros::WallTime::now();
	rosneuro::DynamicMatrix<double> output = flattop->apply(input);
	stop_flattop = ros::WallTime::now();

	ROS_INFO("Flattop applied on data in %9.6f ms", ((stop_flattop-start_flattop).toNSec())/1000.0f);

	// Writing the filtered data
	writeCSV<double>(fileout, output);

	ros::shutdown();
	

	return 0;

}
