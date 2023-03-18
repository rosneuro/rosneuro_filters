#include <ros/ros.h>
#include "rosneuro_filters/rosneuro_filters_utilities.hpp"
#include "rosneuro_filters/Laplacian.hpp"

int main(int argc, char** argv) {


	ros::init(argc, argv, "laplacian_simloop");

	std::string datapath;
	int framesize;
	std::string layout;
	
	rosneuro::Laplacian<double>* laplacian = new rosneuro::Laplacian<double>();

	if(ros::param::get("~datapath", datapath) == false) {
		ROS_ERROR("Cannot find 'datapath' parameter");
		return 0;
	}
	
	if(ros::param::get("~framesize", framesize) == false) {
		ROS_ERROR("Cannot find 'framesize' parameter");
		return 0;
	}
	
	if(ros::param::get("~layout", layout) == false) {
		ROS_ERROR("Cannot find 'layout' parameter");
		return 0;
	}


	const std::string fileinput = datapath + "/test/rawdata.csv";
	const std::string fileout   = datapath + "/test/laplacian_simloop.csv";
	
	// Load input data
	rosneuro::DynamicMatrix<double> input = readCSV<double>(fileinput);

	// Get number of samples and channels
	int nsamples  = input.rows();
	int nchannels = input.cols();
	
	// Configure laplacian filter
	if(laplacian->set_layout(layout, nchannels) == false) {
		ROS_ERROR("[%s] filter configuration failed", laplacian->name().c_str());
		return false;
	}
	ROS_INFO("[%s] filter configuration succeeded", laplacian->name().c_str());

	// Allocate matrix for filtered data
	rosneuro::DynamicMatrix<double> output = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	
	// Allocate frame data (for simulating online loop) 	
	rosneuro::DynamicMatrix<double> framedata = rosneuro::DynamicMatrix<double>::Zero(framesize, nchannels);

	// Allocate time variables
	ros::WallTime start_laplacian, stop_laplacian;
	ros::WallTime start_loop, stop_loop;
	Eigen::VectorXd time_laplacian(nsamples/framesize);

	// Start iteration for simulating online loop
	ROS_INFO("Start simulated loop");
	start_loop = ros::WallTime::now();
	auto count = 0;
	for(auto i = 0; i<nsamples; i = i+framesize) {

		framedata = input.middleRows(i, framesize);
		
		start_laplacian = ros::WallTime::now();
		output.middleRows(i, framesize) = laplacian->apply(framedata);
		stop_laplacian = ros::WallTime::now();
		
		time_laplacian(count) = (stop_laplacian - start_laplacian).toNSec();
		count++;
	}
	stop_loop = ros::WallTime::now();

	Eigen::Index max_id, min_id;
	float mean, max, min, mean_loop;

	mean = time_laplacian.mean()/1000.0f;
	max  = time_laplacian.maxCoeff(&max_id)/1000.0f;
	min  = time_laplacian.minCoeff(&min_id)/1000.0f;
	mean_loop = (stop_loop-start_loop).toNSec()/1000.0f; 
	
	ROS_INFO("Loop ended: filter applied on data");
	ROS_INFO("Car iteration time  | Average: %9.6f ms, Max: %09.6f ms (at %ld), Min: %09.6f ms (at %ld)", 
			 mean, max, max_id, min, min_id); 
	ROS_INFO("Overall loop time (%d iterations): %f ms", count, mean_loop); 

	// Writing the filtered data
	writeCSV<double>(fileout, output);

	ros::shutdown();
	

	return 0;

}
