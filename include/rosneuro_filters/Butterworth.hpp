#ifndef ROSNEURO_FILTERS_BUTTERWORTH_HPP
#define ROSNEURO_FILTERS_BUTTERWORTH_HPP

#include <Eigen/Dense>
#include <rtf_common.h>
#include <rtfilter.h>
#include "rosneuro_filters/Filter.hpp"

namespace rosneuro {

enum class ButterType {Unknown = -1, LowPass, HighPass};

template <typename T>
class Butterworth: public Filter<T> {
	
	public:
		Butterworth(void);
		Butterworth(ButterType type, int order, double cutoff, double samplerate);
		~Butterworth(void);

		bool configure(void);
		DynamicMatrix<T> apply(const DynamicMatrix<T>& in);

		int type(void) const;
		int order(void) const;
		double cutoff(void) const;
		double samplerate(void) const;
			
		void dump(void);

	private:
		bool create_butterworth_filter(int nchannels);
		std::string buttertype(ButterType type);

	private:
		int 	nchannels_;
		double 	samplerate_;
		int		type_;
		int 	order_;
		double 	cutoff_;
		hfilter filter_;
		
		bool is_filter_configured_;
		bool is_filter_set_;
};

template<typename T>
Butterworth<T>::Butterworth(void) {
	this->name_    	  = this->buttertype(ButterType::Unknown);
	this->samplerate_ = 0.0f;
	this->type_       = static_cast<int>(ButterType::Unknown);
	this->order_      = 0;
	this->cutoff_     = 0.0f;
	this->filter_  	  = nullptr;
	
	this->is_filter_configured_ = false;
	this->is_filter_set_ 	    = false;
}

template<typename T>
Butterworth<T>::Butterworth(ButterType type, int order, double cutoff, double samplerate)  {

	this->name_       = this->buttertype(type);
	this->samplerate_ = samplerate;
	this->type_       = static_cast<int>(type);
	this->order_      = order;
	this->cutoff_     = cutoff;
	this->filter_  	  = nullptr;

	this->is_filter_configured_ = true;
	this->is_filter_set_ 	    = false;
}

template<typename T>
Butterworth<T>::~Butterworth(void) {
	if (this->filter_ != nullptr)
		rtf_destroy_filter(this->filter_);
}

template<>
bool Butterworth<float>::create_butterworth_filter(int nchannels) {
	
	bool 	retcod = true;
	double 	normfc;

	this->nchannels_ = nchannels;
		
	normfc = this->cutoff_/this->samplerate_;
	this->filter_ = rtf_create_butterworth(this->nchannels_, RTF_FLOAT, normfc, 
									 this->order_, static_cast<unsigned int>(this->type_));

	if(this->filter_ == nullptr)
			retcod = false;

	return retcod;
}

template<>
bool Butterworth<double>::create_butterworth_filter(int nchannels) {
	
	bool 	retcod = true;
	double 	normfc;

	this->nchannels_ = nchannels;
		
	normfc = this->cutoff_/this->samplerate_;
	this->filter_ = rtf_create_butterworth(this->nchannels_, RTF_DOUBLE, normfc, 
									 this->order_, static_cast<unsigned int>(this->type_));

	if(this->filter_ == nullptr)
			retcod = false;

	return retcod;
}


template<typename T>
DynamicMatrix<T> Butterworth<T>::apply(const DynamicMatrix<T>& in) {
	
	// If the filter is not configured, then throw an exception
	if(this->is_filter_configured_ == false) {
		throw std::runtime_error("[" + this->name() + "] - Filter is not configured");
	}

	// If the filter is not set (rtfilter), then set it the first time the
	// function is called. If it is not possible to set the filter, then throw
	// an exception
	if(this->is_filter_set_ == false) {
		this->is_filter_set_ = this->create_butterworth_filter(in.cols());
	
		if(this->is_filter_set_ == false)
			throw std::runtime_error("[" + this->name() + "] - First apply: cannot set the filter");
		else	
			ROS_WARN("[%s] First apply: the filter is set", this->name().c_str());
	}

	// Allocate input and output matrix (transposed [channels x samples])
	DynamicMatrix<T> infilt = DynamicMatrix<T>::Zero(in.cols(), in.rows());
	DynamicMatrix<T> output = DynamicMatrix<T>::Zero(in.cols(), in.rows());

	// Transpose input 
	infilt = in.transpose();

	// Get pointer to the input data
	T* p_in  = const_cast<T*>( infilt.data());
	
	// Get pointer to the output data
	T* p_out = static_cast<T*>(output.data());

	// Apply filter
	rtf_filter(this->filter_, p_in, p_out, output.cols());

	// Return the transposed version [samples x channels] of the output
	return output.transpose();
}

template<typename T>
bool Butterworth<T>::configure(void) {

	std::string s_type;

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

	if (!Filter<T>::getParam(std::string("type"), s_type)) {
    	ROS_ERROR("[Butterworth] Cannot find param 'type'");
		return false;
	}

	// Case-insensitve
	std::transform(s_type.begin(), s_type.end(), s_type.begin(), ::tolower);

	if (s_type.compare("lowpass") == 0) {
		this->type_ = static_cast<unsigned int>(ButterType::LowPass);
	} else if (s_type.compare("highpass") == 0 ) {
		this->type_ = static_cast<unsigned int>(ButterType::HighPass);
	} else {
		this->type_ = static_cast<unsigned int>(ButterType::Unknown);
		ROS_ERROR("[Butterworth] Unknown butterworth filter type");
		return false;
	}

	return true;
}

template<typename T>
int Butterworth<T>::type(void) const {
	return this->type_;
}

template<typename T>
int Butterworth<T>::order(void) const {
	return this->order_;
}

template<typename T>
double Butterworth<T>::cutoff(void) const {
	return this->cutoff_;
}

template<typename T>
double Butterworth<T>::samplerate(void) const {
	return this->samplerate_;
}

template<typename T> 
std::string Butterworth<T>::buttertype(ButterType type) {
	std::string name;

	switch(type) {
		case ButterType::LowPass:
			name = "lowpass";
			break;
		case ButterType::HighPass:
			name = "highpass";
			break;
		default:
			name = "unknown";
			break;
	}

	return name;
}

template<typename T> 
void Butterworth<T>::dump(void) {

	printf("===================================================\n");
	printf("[%s] - Filter configuration:\n", this->name().c_str());
	printf("===================================================\n");
	printf("\tSamplerate:     %f\n", this->samplerate());
	printf("\tFilter type:    %d\n", this->type());
	printf("\tFilter order:   %d\n", this->order());
	printf("\tFilter cutoff:  %f\n", this->cutoff());

	printf("\n");

}

}


#endif
