#include <ros/ros.h>
#include "rosneuro_data/NeuroData.hpp"
#include "rosneuro_filters/Laplacian.hpp"

int main(int argc, char** argv) {


	ros::init(argc, argv, "test_laplacian");

	constexpr unsigned int nsamples  = 4;
	constexpr unsigned int nchannels = 16;

	rosneuro::NeuroData<float>  in(nsamples, nchannels, "EEG");
	rosneuro::NeuroData<float> out(nsamples, nchannels, "EEG");
	
	rosneuro::Filter<float>* laplacian = new rosneuro::Laplacian<float>();
	if(laplacian->configure("LaplacianCfgTest") == false) {
		ROS_ERROR("Laplacian filter configuration failed");
		return false;
	}
	ROS_INFO("Laplacian filter configuration succeeded");

			
	Eigen::MatrixXf data = Eigen::MatrixXf::Random(nsamples, nchannels);
	std::cout<<data<<std::endl;
	std::copy(data.data(), data.data() + data.size(), in.data());

	if(laplacian->apply(in, out) == false) {
		std::cout<<"Laplacian filter failed"<<std::endl;
		ros::shutdown();
		return 0;
	}
		
	std::cout<<"Laplacian filter succesfully applied"<<std::endl;
	Eigen::Map<Eigen::MatrixXf> eout(out.data(), nsamples, nchannels);

	std::cout<<eout<<std::endl;

	ros::shutdown();
	return 0;

}
