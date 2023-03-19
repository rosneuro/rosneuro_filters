#include <ros/ros.h>
#include "rosneuro_filters/rosneuro_filters_utilities.hpp"
#include "rosneuro_filters/Hamming.hpp"

int main(int argc, char** argv) {

	ros::init(argc, argv, "hamming_window");
	
	std::string datapath;
	
	rosneuro::Filter<double>* hamming = new rosneuro::Hamming<double>();

	if(ros::param::get("~datapath", datapath) == false) {
		ROS_ERROR("Cannot find 'datapath' parameter");
		return 0;
	}
	
	const std::string fileinput = datapath + "/test/rawdata.csv";
	const std::string fileout   = datapath + "/test/hamming_window.csv";
	
	// Load input data
	rosneuro::DynamicMatrix<double> input = readCSV<double>(fileinput);
	
	// Allocate time variables
	ros::WallTime start_hamming, stop_hamming;

	// Apply the filter
	ROS_INFO("Applying filter");
	start_hamming = ros::WallTime::now();
	rosneuro::DynamicMatrix<double> output = hamming->apply(input);
	stop_hamming = ros::WallTime::now();

	ROS_INFO("Hamming applied on data in %9.6f ms", ((stop_hamming-start_hamming).toNSec())/1000.0f);

	// Writing the filtered data
	writeCSV<double>(fileout, output);

	ros::shutdown();
	

	return 0;

}
