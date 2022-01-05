#ifndef ROSNEURO_FILTERS_FLATTOP_HPP
#define ROSNEURO_FILTERS_FLATTOP_HPP

#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"
#include "pluginlib/class_list_macros.h"
  
namespace rosneuro {

template <typename T>
class Flattop : public Filter<T> {
	public:
		Flattop(void) {};
		~Flattop(void) {};

		bool configure(void);
		bool apply(const NeuroData<T>& data_in, NeuroData<T>& data_out);

	private:
		unsigned int nsamples_;
		Eigen::Matrix<T, Eigen::Dynamic, 1> window_;
		T wnorm_;
		static constexpr double A0 = 0.215578950;
    	static constexpr double A1 = 0.416631580;
    	static constexpr double A2 = 0.277263158;
    	static constexpr double A3 = 0.083578947;
    	static constexpr double A4 = 0.006947368;		
		
};

}

#include "../src/Flattop.cpp"

PLUGINLIB_EXPORT_CLASS(rosneuro::Flattop<int>,    rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Flattop<float>,  rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::Flattop<double>, rosneuro::Filter<double>)

#endif
