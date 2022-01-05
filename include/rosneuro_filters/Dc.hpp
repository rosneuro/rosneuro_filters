#ifndef ROSNEURO_FILTERS_DC_HPP
#define ROSNEURO_FILTERS_DC_HPP

#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"
  
namespace rosneuro {

template <typename T>
class Dc : public Filter<T> {
	public:
		Dc(void) {};
		~Dc(void) {};

		bool configure(void);
		bool apply(const NeuroData<T>& data_in, NeuroData<T>& data_out);

};


template<typename T>
bool Dc<T>::configure(void) {
	return true;
}

template<typename T>
bool Dc<T>::apply(const NeuroData<T>& data_in, NeuroData<T>& data_out) {
	T* p_in  = const_cast<T*>(data_in.data());
	T* p_out = const_cast<T*>(data_out.data());

	unsigned int ns_in  = data_in.nsamples();
	unsigned int nc_in  = data_in.nchannels();
	unsigned int ns_out = data_out.nsamples();
	unsigned int nc_out = data_out.nchannels();

	if(ns_in != ns_out) {
		ROS_ERROR("[Dc] Different number of samples between data in and data out");
		return false;
	}
	
	if(nc_in != nc_out) {
		ROS_ERROR("[Dc] Different number of channels between data in and data out");
		return false;
	}

	Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> in(p_in, ns_in, nc_in);
	Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> out(p_out, ns_out, nc_out);

	out = in - (in.colwise().mean()).replicate(in.rows(), 1);

	return true;
}


}

#endif
