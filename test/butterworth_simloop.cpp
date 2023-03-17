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
	const std::string fileoutbp  = datapath + "/test/butterworth_simloop_outbp.csv";

	// Load input data
	rosneuro::DynamicMatrix<double> input = readCSV<double>(fileinput);

	// Get number of samples and channels
	int nsamples  = input.rows();
	int nchannels = input.cols();

	// Allocate matrix for lowpass, highpass, and bandpass data
	rosneuro::DynamicMatrix<double> outlp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	rosneuro::DynamicMatrix<double> outhp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	rosneuro::DynamicMatrix<double> outbp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	
	// Instanciate the lowpass, highpass filters (bandpass is again a highpass
	// filter to be applied on lowpassed data
	rosneuro::Butterworth<double> butter_lp(rosneuro::ButterType::LowPass,  order_lp, cutoff_lp, samplerate);
	rosneuro::Butterworth<double> butter_hp(rosneuro::ButterType::HighPass, order_hp, cutoff_hp, samplerate);
	rosneuro::Butterworth<double> butter_bp_hp(rosneuro::ButterType::HighPass, order_hp, cutoff_hp, samplerate);

	// Dump filter configuration
	butter_lp.dump();
	butter_hp.dump();

	// Allocate frame data (for simulating online loop) 	
	rosneuro::DynamicMatrix<double> framedata = rosneuro::DynamicMatrix<double>::Zero(framesize, nchannels);

	// Start iteration for simulating online loop
	ros::WallTime lstart, lstop;
	ROS_INFO("Start simulated loop");
	lstart = ros::WallTime::now();
	for(auto i = 0; i<nsamples; i = i+framesize) {

		framedata = input.middleRows(i, framesize);

		outlp.middleRows(i, framesize) = butter_lp.apply(framedata);
		outhp.middleRows(i, framesize) = butter_hp.apply(framedata);
		outbp.middleRows(i, framesize) = butter_bp_hp.apply(outlp.middleRows(i, framesize));
	}
	lstop = ros::WallTime::now();

	ROS_INFO("Loop ended in %f s: filters applied on data", (lstop-lstart).toSec());

	// Writing the filtered data
	writeCSV<double>(fileoutlp, outlp);
	writeCSV<double>(fileouthp, outhp);
	writeCSV<double>(fileoutbp, outbp);

	ros::shutdown();
	
	return 0;
}
