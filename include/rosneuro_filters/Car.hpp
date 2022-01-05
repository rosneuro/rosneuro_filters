#ifndef ROSNEURO_FILTERS_CAR_HPP
#define ROSNEURO_FILTERS_CAR_HPP

#include <Eigen/Dense>
#include "Filter.hpp"
#include "pluginlib/class_list_macros.h"

namespace rosneuro {

template <typename T>
class Car : public Filter<T> {
	public:
		Car(void) {};
		~Car(void) {};

		bool configure(void);
		bool apply(const NeuroData<T>& data_in, NeuroData<T>& data_out);

};

}

#include "../src/Car.cpp"

PLUGINLIB_EXPORT_CLASS(rosneuro::Car<int>,    rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Car<float>,  rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Car<double>, rosneuro::Filter<double>)

#endif
