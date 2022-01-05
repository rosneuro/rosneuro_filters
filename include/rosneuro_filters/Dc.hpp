#ifndef ROSNEURO_FILTERS_DC_HPP
#define ROSNEURO_FILTERS_DC_HPP

#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"
#include "pluginlib/class_list_macros.h"
  
namespace rosneuro {

template <typename T>
class Dc : public Filter<T> {
	public:
		Dc(void) {};
		~Dc(void) {};

		bool configure(void);
		bool apply(const NeuroData<T>& data_in, NeuroData<T>& data_out);

};

}

#include "../src/Dc.cpp"

PLUGINLIB_EXPORT_CLASS(rosneuro::Dc<int>,    rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Dc<float>,  rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Dc<double>, rosneuro::Filter<double>)

#endif
