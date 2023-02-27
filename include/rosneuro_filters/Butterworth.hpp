#ifndef ROSNEURO_FILTERS_BUTTERWORTH_HPP
#define ROSNEURO_FILTERS_BUTTERWORTH_HPP

#include <algorithm>
#include <Eigen/Dense>
#include <rtf_common.h>
#include <rtfilter.h>

#include "rosneuro_filters/Filter.hpp"

namespace rosneuro {

enum class ButterType {LOWPASS = 0, HIGHPASS};

template <typename T>
class Butterworth: public Filter<T> {
	
	public:
		Butterworth(void);
		~Butterworth(void);

		bool configure(void);
		bool apply(const NeuroData<T>& data_in, NeuroData<T>& data_out);

	private:
		bool setup_rtfilter(void);

	private:
		unsigned int order_;
		unsigned int nchannels_;
		double 		 samplerate_;
		double 		 cutoff_;
		unsigned int type_;
		hfilter 	 filt_ = nullptr;
};

template<typename T>
Butterworth<T>::Butterworth(void) {
}

template<typename T>
Butterworth<T>::~Butterworth(void) {
	if (this->filt_ != nullptr)
		rtf_destroy_filter(this->filt_);
}


template<typename T>
bool Butterworth<T>::configure(void) {

	std::string stype;
	bool retcod;

	if (!Filter<T>::getParam(std::string("nchannels"), this->nchannels_)) {
    	ROS_ERROR("[Butterworth] Cannot find param 'nchannels'");
		return false;
	}
	
	if (!Filter<T>::getParam(std::string("order"), this->order_)) {
    	ROS_ERROR("[Butterworth] Cannot find param 'order'");
		return false;
	}
	
	if (!Filter<T>::getParam(std::string("samplerate"), this->samplerate_)) {
    	ROS_ERROR("[Butterworth] Cannot find param 'samplerate'");
		return false;
	}
	
	if (!Filter<T>::getParam(std::string("cutoff"), this->cutoff_)) {
    	ROS_ERROR("[Butterworth] Cannot find param 'cutoff'");
		return false;
	}

	if (!Filter<T>::getParam(std::string("type"), stype)) {
    	ROS_ERROR("[Butterworth] Cannot find param 'type'");
		return false;
	}

	std::transform(stype.begin(), stype.end(), stype.begin(), ::tolower);

	if (stype.compare("lowpass") == 0) {
		this->type_ = static_cast<unsigned int>(ButterType::LOWPASS);
	} else if (stype.compare("highpass") == 0 ) {
		this->type_ = static_cast<unsigned int>(ButterType::HIGHPASS);
	} else if (stype.compare("bandpass") == 0 ) {
		ROS_ERROR("[Butterworth] Butterworth bandpass filter not implemented yet. Use a sequence of low and highpass filters");
		return false;
		//this->type_ = static_cast<unsigned int>(ButterType::BANDPASS);
	} else {
		ROS_ERROR("[Butterworth] Unknown butterworth filter type");
		return false;
	}

	// Setup filter
	retcod = this->setup_rtfilter();

	return retcod;
	
}

template<typename T>
bool Butterworth<T>::apply(const NeuroData<T>& data_in, NeuroData<T>& data_out) {

	T* p_in  = const_cast<T*>(data_in.data());
	T* p_out = const_cast<T*>(data_out.data());

	unsigned int ns_in  = data_in.nsamples();
	unsigned int nc_in  = data_in.nchannels();
	unsigned int ns_out = data_out.nsamples();
	unsigned int nc_out = data_out.nchannels();


	if(ns_in != ns_out) {
		ROS_ERROR("[Butterworth] Different number of samples between data in and data out");
		return false;
	}
	
	if(nc_in != nc_out) {
		ROS_ERROR("[Butterworth] Different number of channels between data in and data out");
		return false;
	}

	rtf_filter(this->filt_, p_in, p_out, ns_in);
	
	return true;
}

template<>
bool Butterworth<float>::setup_rtfilter(void) {

	bool retcod = true;
	double normfc;

	normfc = this->cutoff_/this->samplerate_;
	this->filt_ = rtf_create_butterworth(this->nchannels_, RTF_FLOAT, normfc, 
										 this->order_, static_cast<unsigned int>(this->type_));

	if(this->filt_ == nullptr)
		retcod = false;

	return retcod;
}

template<>
bool Butterworth<double>::setup_rtfilter(void) {

	bool retcod = true;

	double normfc;
	normfc = this->cutoff_/this->samplerate_;
	this->filt_ = rtf_create_butterworth(this->nchannels_, RTF_DOUBLE, normfc, 
										 this->order_, static_cast<unsigned int>(this->type_));

	if(this->filt_ == nullptr)
		retcod = false;

	return retcod;
}

}


#endif
