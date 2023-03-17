#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include "rosneuro_filters/rosneuro_filters_utilities.hpp"
#include "rosneuro_filters/Butterworth.hpp"


int main(int argc, char** argv) {


	std::string fileinput  = "/mnt/data/workspaces/rosneuro_filters_ws/src/rosneuro_filters/extra/input.csv";
	std::string fileoutlp  = "/mnt/data/workspaces/rosneuro_filters_ws/src/rosneuro_filters/extra/outlp.csv";
	std::string fileouthp  = "/mnt/data/workspaces/rosneuro_filters_ws/src/rosneuro_filters/extra/outhp.csv";
	std::string fileoutbp  = "/mnt/data/workspaces/rosneuro_filters_ws/src/rosneuro_filters/extra/outbp.csv";


	ros::init(argc, argv, "test_butterworth");

	rosneuro::DynamicMatrix<double> input   = readCSV<double>(fileinput);
	rosneuro::DynamicMatrix<double> inputlp = readCSV<double>(fileinput);
	rosneuro::DynamicMatrix<double> inputhp = readCSV<double>(fileinput);

	unsigned int nsamples  = input.rows();
	unsigned int nchannels = input.cols();
	unsigned int chunk = 32;

	rosneuro::DynamicMatrix<double> outlp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	rosneuro::DynamicMatrix<double> outhp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	rosneuro::DynamicMatrix<double> outbp = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);

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

	//int i = 0;
	//rosneuro::DynamicMatrix<double> buffer, tmplp, tmphp, tmpbp;
	//while(i<nsamples-buffersize) {
	//	buffer = input.middleRows(i, buffersize);
	//	tmplp = butter_lp->apply(buffer);
	//	tmphp = butter_hp->apply(buffer);

	//	if(i == 0) {
	//		outlow.topRows(buffersize) = tmplp;
	//		outhigh.topRows(buffersize) = tmphp;
	//	} else {
	//		outlow.middleRows(i + buffersize - chunk, chunk) = tmplp.bottomRows(chunk);
	//		outhigh.middleRows(i + buffersize - chunk, chunk) = tmphp.bottomRows(chunk);
	//	}

	//	
	//	i = i + chunk;
	//}


	int i = 0;
	while(i<nsamples) {
	//while(i<chunk*17) {
		outlp.middleRows(i, chunk) = butter_lp->apply(inputlp.middleRows(i, chunk));
		outhp.middleRows(i, chunk) = butter_hp->apply(inputhp.middleRows(i, chunk));
		outbp.middleRows(i, chunk) = butter_lp->apply(outhp.middleRows(i, chunk));
		i = i + chunk;
	}
	
	//outlow = butter_lp->apply(input);
	//outhigh = butter_hp->apply(input);
	//outbp  = butter_lp->apply(outhigh);

	ROS_INFO("Bandpass filter applied");

	//writeMatrix(output, fileout.c_str());
	writeCSV<double>(fileoutlp, outlp);
	writeCSV<double>(fileouthp, outhp);
	writeCSV<double>(fileoutbp, outbp);
	ros::shutdown();

	
	return 0;

}
