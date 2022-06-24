#include <ros/ros.h>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_filters/Butterworth.hpp"

int main(int argc, char** argv) {


	ros::init(argc, argv, "test_butterworth");

	constexpr unsigned int nsamples  = 512;
	constexpr unsigned int nchannels = 1;

	rosneuro::NeuroData<float>  in(nsamples, nchannels, "EEG");
	rosneuro::NeuroData<float> out(nsamples, nchannels, "EEG");
	
	rosneuro::Filter<float>* butter = new rosneuro::Butterworth<float>();
	if(butter->configure("ButterworthCfgTest") == false) {
		ROS_ERROR("Butterworth filter configuration failed");
		return false;
	}
	ROS_INFO("Butterworth filter configuration succeeded");

			
	Eigen::MatrixXf data = Eigen::MatrixXf::Random(nsamples, nchannels);
	//std::cout<<"Original data"<<std::endl;
	//std::cout<<data<<std::endl;
	std::copy(data.data(), data.data() + data.size(), in.data());

	if(butter->apply(in, out) == false) {
		printf("here\n");
		std::cout<<"Butterworth filter failed"<<std::endl;
		ros::shutdown();
		return 0;
	}
		
	std::cout<<"Butterworth filter succesfully applied"<<std::endl;
	//Eigen::Map<Eigen::MatrixXf> eout(out.data(), nsamples, nchannels);

	//std::cout<<eout<<std::endl;

	printf("here0\n");
	//delete butter;
	printf("here1\n");
	ros::shutdown();

	printf("here2\n");
	
	return 0;

}
