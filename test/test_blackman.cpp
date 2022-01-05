#include <ros/ros.h>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_filters/Blackman.hpp"

int main(int argc, char** argv) {


	ros::init(argc, argv, "test_blackman");
	
	rosneuro::Filter<float>* blackman = new rosneuro::Blackman<float>();
	if(blackman->configure("BlackmanCfgTest") == false) {
		ROS_ERROR("Blackman window configuration failed");
		return false;
	}
	ROS_INFO("Blackman window configuration succeeded");

	rosneuro::NeuroData<float> in(4, 3, "EEG");
	rosneuro::NeuroData<float> out(4, 3, "EEG");
			
	Eigen::MatrixXf data = Eigen::MatrixXf::Random(4, 3);
	std::cout<<data<<std::endl;
	std::copy(data.data(), data.data() + data.size(), in.data());

	if(blackman->apply(in, out) == false) {
		std::cout<<"Blackman window failed"<<std::endl;
		ros::shutdown();
		return 0;
	}
		
	std::cout<<"Blackman window succesfully applied"<<std::endl;
	Eigen::Map<Eigen::MatrixXf> eout(out.data(), 4, 3);

	std::cout<<eout<<std::endl;

	delete blackman;

	ros::shutdown();
	return 0;

}
