#ifndef ROSNEURO_FILTERS_BLACKMAN_HPP
#define ROSNEURO_FILTERS_BLACKMAN_HPP

#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"
#include "pluginlib/class_list_macros.h"
  
namespace rosneuro {

template <typename T>
class Blackman : public Filter<T> {
	public:
		Blackman(void) {};
		~Blackman(void) {};

		bool configure(void);
		bool apply(const NeuroData<T>& data_in, NeuroData<T>& data_out);

	private:
		unsigned int nsamples_;
		Eigen::Matrix<T, Eigen::Dynamic, 1> window_;
		T wnorm_;
		
};

}

#include "../src/Blackman.cpp"

PLUGINLIB_EXPORT_CLASS(rosneuro::Blackman<int>,    rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Blackman<float>,  rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Blackman<double>, rosneuro::Filter<double>)

#endif
