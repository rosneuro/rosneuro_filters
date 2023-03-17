#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include "rosneuro_filters/rosneuro_filters_utilities.hpp"
#include "rosneuro_filters/Butterworth.hpp"
#include "rosneuro_filters/FilterChain.hpp"

int main(int argc, char** argv) {

	ros::init(argc, argv, "butterworth_simloop_chain");
	
	std::string datapath;
	int framesize;
	
	// YAML configuration for the filter chain
	rosneuro::FilterChain<double> butter_bp;
	if(butter_bp.configure("ButterworthBandPass") == false) {
		ROS_ERROR("[FilterChain] Bandpass filter configuration failed");
		return false;
	}
	ROS_INFO("[FilterChain] Bandpass filter configuration succeeded");
	
	if(ros::param::get("~datapath", datapath) == false) {
		ROS_ERROR("Cannot find 'datapath' parameter");
		return 0;
	}
	
	if(ros::param::get("~framesize", framesize) == false) {
		ROS_ERROR("Cannot find 'framesize' parameter");
		return 0;
	}

	const std::string fileinput  = datapath + "/test/input.csv";
	const std::string fileoutbp  = datapath + "/test/butterworth_simloop_chain_outbp.csv";

	// Load input data
	rosneuro::DynamicMatrix<double> input = readCSV<double>(fileinput);
	
	butter_bp.dump();

	// Get number of samples and channels
	int nsamples  = input.rows();
	int nchannels = input.cols();

	// Allocate matrix for lowpass, highpass, and bandpass data
	rosneuro::DynamicMatrix<double> outbp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	
	// Allocate frame data (for simulating online loop) 	
	rosneuro::DynamicMatrix<double> framedata = rosneuro::DynamicMatrix<double>::Zero(framesize, nchannels);

	// Allocate time variables
	ros::WallTime start_bp, stop_bp;
	ros::WallTime start_loop, stop_loop;
	Eigen::VectorXd time_bp(nsamples/framesize);
	
	// Start iteration for simulating online loop
	ROS_INFO("Start simulated loop");
	start_loop = ros::WallTime::now();
	auto count = 0;
	for(auto i = 0; i<nsamples; i = i+framesize) {

		framedata = input.middleRows(i, framesize);
		
		start_bp = ros::WallTime::now();
		outbp.middleRows(i, framesize) = butter_bp.apply(framedata);
		stop_bp = ros::WallTime::now();
		
		time_bp(count) = (stop_bp - start_bp).toNSec();
		count++;
	}
	stop_loop = ros::WallTime::now();
	
	Eigen::Index max_id_bp, min_id_bp;
	float mean_bp, mean_loop;
	float max_bp, min_bp;

	mean_bp   = time_bp.mean()/1000.0f;
	mean_loop = (stop_loop-start_loop).toNSec()/1000.0f; 
	max_bp = time_bp.maxCoeff(&max_id_bp)/1000.0f;
	min_bp = time_bp.minCoeff(&min_id_bp)/1000.0f;
	
	ROS_INFO("Loop ended: filters applied on data");
	ROS_INFO("Band-pass iteration time  | Average: %9.6f ms, Max: %09.6f ms (at %ld), Min: %09.6f ms (at %ld)", 
			 mean_bp, max_bp, max_id_bp, min_bp, min_id_bp); 
	ROS_INFO("Overall loop time (%d iterations): %f ms", count, mean_loop); 


	// Writing the filtered data
	writeCSV<double>(fileoutbp, outbp);

	ros::shutdown();
	
	return 0;
}
