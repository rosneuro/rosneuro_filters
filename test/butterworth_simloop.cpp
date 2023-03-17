#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include "rosneuro_filters/rosneuro_filters_utilities.hpp"
#include "rosneuro_filters/Butterworth.hpp"
#include <unistd.h>

int main(int argc, char** argv) {

	std::string datapath;
	
	double samplerate, cutoff_lp, cutoff_hp;
	int framesize, order_lp, order_hp;
	
	ros::init(argc, argv, "butterworth_simloop");
	if(ros::param::get("~datapath", datapath) == false) {
		ROS_ERROR("Cannot find 'datapath' parameter");
		return 0;
	}
	if(ros::param::get("~samplerate", samplerate) == false) {
		ROS_ERROR("Cannot find 'samplerate' parameter");
		return 0;
	}
	if(ros::param::get("~order_lp", order_lp) == false) {
		ROS_ERROR("Cannot find 'order_lp' parameter");
		return 0;
	}
	if(ros::param::get("~order_hp", order_hp) == false) {
		ROS_ERROR("Cannot find 'order_hp' parameter");
		return 0;
	}
	if(ros::param::get("~cutoff_lp", cutoff_lp) == false) {
		ROS_ERROR("Cannot find 'cutoff_lp' parameter");
		return 0;
	}
	if(ros::param::get("~cutoff_hp", cutoff_hp) == false) {
		ROS_ERROR("Cannot find 'cutoff_hp' parameter");
		return 0;
	}
	if(ros::param::get("~framesize", framesize) == false) {
		ROS_ERROR("Cannot find 'framesize' parameter");
		return 0;
	}

	const std::string fileinput  = datapath + "/test/input.csv";
	const std::string fileoutlp  = datapath + "/test/butterworth_simloop_outlp.csv";
	const std::string fileouthp  = datapath + "/test/butterworth_simloop_outhp.csv";

	// Load input data
	rosneuro::DynamicMatrix<double> input = readCSV<double>(fileinput);

	// Get number of samples and channels
	int nsamples  = input.rows();
	int nchannels = input.cols();

	// Allocate matrix for lowpass, highpass, and bandpass data
	rosneuro::DynamicMatrix<double> outlp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	rosneuro::DynamicMatrix<double> outhp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	
	// Instanciate the lowpass, highpass filters (bandpass is again a highpass
	// filter to be applied on lowpassed data
	rosneuro::Butterworth<double> butter_lp(rosneuro::ButterType::LowPass,  order_lp, cutoff_lp, samplerate);
	rosneuro::Butterworth<double> butter_hp(rosneuro::ButterType::HighPass, order_hp, cutoff_hp, samplerate);

	// Dump filter configuration
	butter_lp.dump();
	butter_hp.dump();

	// Allocate frame data (for simulating online loop) 	
	rosneuro::DynamicMatrix<double> framedata = rosneuro::DynamicMatrix<double>::Zero(framesize, nchannels);

	// Allocate time variables
	ros::WallTime start_lp, stop_lp, start_hp, stop_hp;
	ros::WallTime start_loop, stop_loop;
	Eigen::VectorXd time_lp(nsamples/framesize);
	Eigen::VectorXd time_hp(nsamples/framesize); 
	
	// Start iteration for simulating online loop
	ROS_INFO("Start simulated loop");
	start_loop = ros::WallTime::now();
	auto count = 0;
	for(auto i = 0; i<nsamples; i = i+framesize) {

		framedata = input.middleRows(i, framesize);
		
		start_lp = ros::WallTime::now();
		outlp.middleRows(i, framesize) = butter_lp.apply(framedata);
		stop_lp = ros::WallTime::now();
		
		start_hp = ros::WallTime::now();
		outhp.middleRows(i, framesize) = butter_hp.apply(framedata);
		stop_hp = ros::WallTime::now();
		
		time_lp(count) = (stop_lp - start_lp).toNSec();
		time_hp(count) = (stop_hp - start_hp).toNSec();
		count++;
	}
	stop_loop = ros::WallTime::now();


	Eigen::Index max_id_lp, min_id_lp, max_id_hp, min_id_hp;
	float mean_lp, mean_hp, mean_loop;
	float max_lp, min_lp, max_hp, min_hp;

	mean_lp   = time_lp.mean()/1000.0f;
	mean_hp   = time_hp.mean()/1000.0f;
	mean_loop = (stop_loop-start_loop).toNSec()/1000.0f; 
	max_lp = time_lp.maxCoeff(&max_id_lp)/1000.0f;
	max_hp = time_hp.maxCoeff(&max_id_hp)/1000.0f;
	min_lp = time_lp.minCoeff(&min_id_lp)/1000.0f;
	min_hp = time_hp.minCoeff(&min_id_hp)/1000.0f;
	
	ROS_INFO("Loop ended: filters applied on data");
	ROS_INFO("Low-pass iteration time  | Average: %9.6f ms, Max: %09.6f ms (at %ld), Min: %09.6f ms (at %ld)", 
			 mean_lp, max_lp, max_id_lp, min_lp, min_id_lp); 
	ROS_INFO("High-pass iteration time | Average: %9.6f ms, Max: %09.6f ms (at %ld), Min: %09.6f ms (at %ld)", 
			 mean_hp, max_hp, max_id_hp, min_hp, min_id_hp); 
	ROS_INFO("Overall loop time (%d iterations): %f ms", count, mean_loop); 

	// Writing the filtered data
	writeCSV<double>(fileoutlp, outlp);
	writeCSV<double>(fileouthp, outhp);

	ros::shutdown();
	
	return 0;
}
