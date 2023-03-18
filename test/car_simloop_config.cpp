#include <ros/ros.h>
#include "rosneuro_filters/rosneuro_filters_utilities.hpp"
#include "rosneuro_filters/Car.hpp"

int main(int argc, char** argv) {

	ros::init(argc, argv, "car_simloop_config");
	
	std::string datapath;
	int framesize;
	
	rosneuro::Filter<double>* car = new rosneuro::Car<double>();
	if(car->configure("Car") == false) {
		ROS_ERROR("[%s] filter configuration failed", car->name().c_str());
		return false;
	}
	ROS_INFO("[%s] filter configuration succeeded", car->name().c_str());

	if(ros::param::get("~datapath", datapath) == false) {
		ROS_ERROR("Cannot find 'datapath' parameter");
		return 0;
	}
	
	if(ros::param::get("~framesize", framesize) == false) {
		ROS_ERROR("Cannot find 'framesize' parameter");
		return 0;
	}

	const std::string fileinput = datapath + "/test/rawdata.csv";
	const std::string fileout   = datapath + "/test/car_simloop_config.csv";
	
	// Load input data
	rosneuro::DynamicMatrix<double> input = readCSV<double>(fileinput);

	// Get number of samples and channels
	int nsamples  = input.rows();
	int nchannels = input.cols();

	// Allocate matrix for filtered data
	rosneuro::DynamicMatrix<double> output = rosneuro::DynamicMatrix<double>::Zero(nsamples, nchannels);
	
	// Allocate frame data (for simulating online loop) 	
	rosneuro::DynamicMatrix<double> framedata = rosneuro::DynamicMatrix<double>::Zero(framesize, nchannels);

	// Allocate time variables
	ros::WallTime start_car, stop_car;
	ros::WallTime start_loop, stop_loop;
	Eigen::VectorXd time_car(nsamples/framesize);

	// Start iteration for simulating online loop
	ROS_INFO("Start simulated loop");
	start_loop = ros::WallTime::now();
	auto count = 0;
	for(auto i = 0; i<nsamples; i = i+framesize) {

		framedata = input.middleRows(i, framesize);
		
		start_car = ros::WallTime::now();
		output.middleRows(i, framesize) = car->apply(framedata);
		stop_car = ros::WallTime::now();
		
		time_car(count) = (stop_car - start_car).toNSec();
		count++;
	}
	stop_loop = ros::WallTime::now();

	Eigen::Index max_id, min_id;
	float mean, max, min, mean_loop;

	mean = time_car.mean()/1000.0f;
	max  = time_car.maxCoeff(&max_id)/1000.0f;
	min  = time_car.minCoeff(&min_id)/1000.0f;
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
