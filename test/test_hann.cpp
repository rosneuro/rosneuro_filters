#include <ros/ros.h>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_filters/Hann.hpp"

int main(int argc, char** argv) {


	ros::init(argc, argv, "test_hann");
	
	rosneuro::Filter<float>* hann = new rosneuro::Hann<float>();
	if(hann->configure("HannCfgTest") == false) {
		ROS_ERROR("Hann window configuration failed");
		return false;
	}
	ROS_INFO("Hann window configuration succeeded");

	rosneuro::NeuroData<float> in(4, 3, "EEG");
	rosneuro::NeuroData<float> out(4, 3, "EEG");
			
	Eigen::MatrixXf data = Eigen::MatrixXf::Random(4, 3);
	std::cout<<data<<std::endl;
	std::copy(data.data(), data.data() + data.size(), in.data());

	if(hann->apply(in, out) == false) {
		std::cout<<"Hann window failed"<<std::endl;
		ros::shutdown();
		return 0;
	}
		
	std::cout<<"Hann window succesfully applied"<<std::endl;
	Eigen::Map<Eigen::MatrixXf> eout(out.data(), 4, 3);

	std::cout<<eout<<std::endl;

	ros::shutdown();
	return 0;

}
