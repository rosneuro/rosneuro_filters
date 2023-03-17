#include <ros/ros.h>
//#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_filters/Butterworth.hpp"

int main(int argc, char** argv) {


	ros::init(argc, argv, "test_butterworth");

	constexpr unsigned int nrows  = 512;
	constexpr unsigned int ncols  = 2;

	rosneuro::Filter<float>* butter = new rosneuro::Butterworth<float>();
	if(butter->configure("ButterworthCfgTest") == false) {
		ROS_ERROR("Butterworth filter configuration failed");
		return false;
	}
	ROS_INFO("Butterworth filter configuration succeeded");

			
	rosneuro::DynamicMatrix<float> in = rosneuro::DynamicMatrix<float>::Random(nrows, ncols);

	rosneuro::DynamicMatrix<float> out;
	out = butter->apply(in);
		
	std::cout<<"Butterworth filter succesfully applied"<<std::endl;

	ros::shutdown();
	
	return 0;

}
