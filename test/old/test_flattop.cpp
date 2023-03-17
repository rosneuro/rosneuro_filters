#include <ros/ros.h>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_filters/Flattop.hpp"

int main(int argc, char** argv) {


	ros::init(argc, argv, "test_flattop");
	
	rosneuro::Filter<float>* flattop = new rosneuro::Flattop<float>();
	if(flattop->configure("FlattopCfgTest") == false) {
		ROS_ERROR("Flattop window configuration failed");
		return false;
	}
	ROS_INFO("Flattop window configuration succeeded");

	rosneuro::NeuroData<float> in(4, 3, "EEG");
	rosneuro::NeuroData<float> out(4, 3, "EEG");
			
	Eigen::MatrixXf data = Eigen::MatrixXf::Random(4, 3);
	std::cout<<data<<std::endl;
	std::copy(data.data(), data.data() + data.size(), in.data());

	if(flattop->apply(in, out) == false) {
		std::cout<<"Flattop window failed"<<std::endl;
		ros::shutdown();
		return 0;
	}
		
	std::cout<<"Flattop window succesfully applied"<<std::endl;
	Eigen::Map<Eigen::MatrixXf> eout(out.data(), 4, 3);

	std::cout<<eout<<std::endl;

	delete flattop;

	ros::shutdown();
	return 0;

}
