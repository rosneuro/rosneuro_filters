#ifndef ROSNEURO_FILTERS_HANN_HPP
#define ROSNEURO_FILTERS_HANN_HPP

#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"
#include "pluginlib/class_list_macros.h"
  
namespace rosneuro {

template <typename T>
class Hann : public Filter<T> {
	public:
		Hann(void) {};
		~Hann(void) {};

		bool configure(void);
		bool apply(const NeuroData<T>& data_in, NeuroData<T>& data_out);

	private:
		unsigned int nsamples_;
		Eigen::Matrix<T, Eigen::Dynamic, 1> window_;
		T wnorm_;
};

}

#include "../src/Hann.cpp"

PLUGINLIB_EXPORT_CLASS(rosneuro::Hann<int>,    rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Hann<float>,  rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Hann<double>, rosneuro::Filter<double>)

#endif
