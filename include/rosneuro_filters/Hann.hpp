#ifndef ROSNEURO_FILTERS_HANN_HPP
#define ROSNEURO_FILTERS_HANN_HPP

#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"
  
namespace rosneuro {

template <typename T>
class Hann : public Filter<T> {
	public:
		Hann(void);
		~Hann(void) {};

		bool configure(void);
		DynamicMatrix<T> apply(const DynamicMatrix<T>& in);

	private:
		bool create_window(int nsamples);

	private:
		int nsamples_;
		bool is_window_set_;
		Eigen::Matrix<T, Eigen::Dynamic, 1> window_;
		T wnorm_;
};

template<typename T>
Hann<T>::Hann(void) {
	this->name_ 	     = "hann";
	this->is_window_set_ = false;
}

template<typename T>
bool Hann<T>::configure(void) {
	return true;
}

template<typename T>
DynamicMatrix<T> Hann<T>::apply(const DynamicMatrix<T>& in) {

	if(this->is_window_set_ == false) {
		this->is_window_set_ = this->create_window(in.rows());
		
		if(this->is_window_set_ == false)
			throw std::runtime_error("[" + this->name() + "] - First apply: cannot create the window");
		else	
			ROS_WARN("[%s] First apply: the window is set", this->name().c_str());
	}
	
	return in.array() * this->window_.replicate(1, in.cols()).array();
}

template<typename T>
bool Hann<T>::create_window(int nsamples) {

	this->nsamples_ = nsamples;

	this->window_ = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(this->nsamples_);

	for(auto i = 0; i<this->nsamples_; i++)
		this->window_(i)   = (1.0 - cos((2.0 * M_PI * i)/(this->nsamples_-1))) * 0.5;
	
	this->wnorm_ = (this->window_.array().pow(2)).sum() / this->window_.size();

	return true;
}

}


#endif
