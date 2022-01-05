#ifndef ROSNEURO_FILTERS_HAMMING_HPP
#define ROSNEURO_FILTERS_HAMMING_HPP

#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"
#include "pluginlib/class_list_macros.h"
  
namespace rosneuro {

template <typename T>
class Hamming : public Filter<T> {
	public:
		Hamming(void) {};
		~Hamming(void) {};

		bool configure(void);
		bool apply(const NeuroData<T>& data_in, NeuroData<T>& data_out);

	private:
		unsigned int nsamples_;
		Eigen::Matrix<T, Eigen::Dynamic, 1> window_;
		T wnorm_;
};

}

#include "../src/Hamming.cpp"

PLUGINLIB_EXPORT_CLASS(rosneuro::Hamming<int>,    rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Hamming<float>,  rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Hamming<double>, rosneuro::Filter<double>)

#endif
