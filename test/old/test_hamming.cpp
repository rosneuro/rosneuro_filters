#include <ros/ros.h>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_filters/Hamming.hpp"

int main(int argc, char** argv) {


	ros::init(argc, argv, "test_hamming");
	
	rosneuro::Filter<float>* hamming = new rosneuro::Hamming<float>();
	if(hamming->configure("HammingCfgTest") == false) {
		ROS_ERROR("Hamming window configuration failed");
		return false;
	}
	ROS_INFO("Hamming window configuration succeeded");

	rosneuro::NeuroData<float> in(4, 3, "EEG");
	rosneuro::NeuroData<float> out(4, 3, "EEG");
			
	Eigen::MatrixXf data = Eigen::MatrixXf::Random(4, 3);
	std::cout<<data<<std::endl;
	std::copy(data.data(), data.data() + data.size(), in.data());

	if(hamming->apply(in, out) == false) {
		std::cout<<"Hamming window failed"<<std::endl;
		ros::shutdown();
		return 0;
	}
		
	std::cout<<"Hamming window succesfully applied"<<std::endl;
	Eigen::Map<Eigen::MatrixXf> eout(out.data(), 4, 3);

	std::cout<<eout<<std::endl;

	ros::shutdown();
	return 0;

}
