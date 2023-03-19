#include <ros/ros.h>
#include "rosneuro_filters/rosneuro_filters_utilities.hpp"
#include "rosneuro_filters/Hann.hpp"

int main(int argc, char** argv) {

	ros::init(argc, argv, "hann_window");
	
	std::string datapath;
	
	rosneuro::Filter<double>* hann = new rosneuro::Hann<double>();

	if(ros::param::get("~datapath", datapath) == false) {
		ROS_ERROR("Cannot find 'datapath' parameter");
		return 0;
	}
	
	const std::string fileinput = datapath + "/test/rawdata.csv";
	const std::string fileout   = datapath + "/test/hann_window.csv";
	
	// Load input data
	rosneuro::DynamicMatrix<double> input = readCSV<double>(fileinput);
	
	// Allocate time variables
	ros::WallTime start_hann, stop_hann;

	// Apply the filter
	ROS_INFO("Applying filter");
	start_hann = ros::WallTime::now();
	rosneuro::DynamicMatrix<double> output = hann->apply(input);
	stop_hann = ros::WallTime::now();

	ROS_INFO("Hann applied on data in %9.6f ms", ((stop_hann-start_hann).toNSec())/1000.0f);

	// Writing the filtered data
	writeCSV<double>(fileout, output);

	ros::shutdown();
	

	return 0;

}
