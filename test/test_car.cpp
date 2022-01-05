#include <ros/ros.h>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_filters/Car.hpp"

int main(int argc, char** argv) {

	ros::init(argc, argv, "test_car");
	
	rosneuro::Filter<float>* car = new rosneuro::Car<float>();
	if(car->configure("CarCfgTest") == false) {
		ROS_ERROR("Car filter configuration failed");
		return false;
	}
	ROS_INFO("Car filter configuration succeeded");

	rosneuro::NeuroData<float> in(2, 3, "EEG");
	rosneuro::NeuroData<float> out(2, 3, "EEG");
			
	Eigen::MatrixXf data = Eigen::MatrixXf::Random(2, 3);
	std::cout<<data<<std::endl;
	std::copy(data.data(), data.data() + data.size(), in.data());

	if(car->apply(in, out) == false) {
		std::cout<<"Car filter failed"<<std::endl;
		ros::shutdown();
		return -1;
	}
		
	std::cout<<"Car filter succesfully applied"<<std::endl;
	Eigen::Map<Eigen::MatrixXf> eout(out.data(), 2, 3);

	std::cout<<eout<<std::endl;

	return 0;

}
