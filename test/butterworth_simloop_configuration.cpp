#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include "rosneuro_filters/rosneuro_filters_utilities.hpp"
#include "rosneuro_filters/Butterworth.hpp"
#include <unistd.h>

int main(int argc, char** argv) {

	ros::init(argc, argv, "butterworth_simloop");
	
	std::string datapath;
	int framesize;
	
	// YAML configuration for the filters
	rosneuro::Filter<double>* butter_lp = new rosneuro::Butterworth<double>();
	if(butter_lp->configure("ButterworthLowPass") == false) {
		ROS_ERROR("[%s] Lowpass filter configuration failed", butter_lp->name().c_str());
		return false;
	}
	ROS_INFO("[%s] Lowpass filter configuration succeeded", butter_lp->name().c_str());
	
	rosneuro::Filter<double>* butter_hp = new rosneuro::Butterworth<double>();
	if(butter_hp->configure("ButterworthHighPass") == false) {
		ROS_ERROR("[%s] Highpass filter configuration failed", butter_hp->name().c_str());
		return false;
	}
	ROS_INFO("[%s] Highpass filter configuration succeeded", butter_hp->name().c_str());
	
	if(ros::param::get("~datapath", datapath) == false) {
		ROS_ERROR("Cannot find 'datapath' parameter");
		return 0;
	}
	
	if(ros::param::get("~framesize", framesize) == false) {
		ROS_ERROR("Cannot find 'framesize' parameter");
		return 0;
	}

	const std::string fileinput  = datapath + "/test/input.csv";
	const std::string fileoutlp  = datapath + "/test/butterworth_simloop_configuration_outlp.csv";
	const std::string fileouthp  = datapath + "/test/butterworth_simloop_configuration_outhp.csv";

	// Load input data
	rosneuro::DynamicMatrix<double> input = readCSV<double>(fileinput);

	// Get number of samples and channels
	int nsamples  = input.rows();
	int nchannels = input.cols();

	// Allocate matrix for lowpass, highpass, and bandpass data
	rosneuro::DynamicMatrix<double> outlp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	rosneuro::DynamicMatrix<double> outhp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	
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
	for(auto i = 0; i<nsamples; i = i+framesize) {

		framedata = input.middleRows(i, framesize);
		
		start_lp = ros::WallTime::now();
		outlp.middleRows(i, framesize) = butter_lp->apply(framedata);
		stop_lp = ros::WallTime::now();
		
		start_hp = ros::WallTime::now();
		outhp.middleRows(i, framesize) = butter_hp->apply(framedata);
		stop_hp = ros::WallTime::now();
		
		time_lp << (stop_lp - start_lp).toNSec();
		time_hp << (stop_hp - start_hp).toNSec();
	}
	stop_loop = ros::WallTime::now();


	ROS_INFO("Loop ended: filters applied on data");
	ROS_INFO("Low-pass iteration time  | Average: %f ns, Max: %f ns, Min: %f ns", 
			 time_lp.mean(), time_lp.maxCoeff(), time_lp.minCoeff()); 
	ROS_INFO("High-pass iteration time | Average: %f ns, Max: %f ns, Min: %f ns", 
			 time_hp.mean(), time_hp.maxCoeff(), time_hp.minCoeff()); 
	ROS_INFO("Overall loop time (%d iterations): %f ms", nsamples/framesize,
														 (stop_loop-start_loop).toNSec()/1000.0f); 

	// Writing the filtered data
	writeCSV<double>(fileoutlp, outlp);
	writeCSV<double>(fileouthp, outhp);

	ros::shutdown();
	
	return 0;
}
