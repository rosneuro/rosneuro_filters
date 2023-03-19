#ifndef ROSNEURO_FILTERS_HAMMING_HPP
#define ROSNEURO_FILTERS_HAMMING_HPP

#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"
  
namespace rosneuro {

template <typename T>
class Hamming : public Filter<T> {
	public:
		Hamming(void);
		~Hamming(void) {};

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
Hamming<T>::Hamming(void) {
	this->name_ 	     = "blackman";
	this->is_window_set_ = false;
}

template<typename T>
bool Hamming<T>::configure(void) {
	return true;
}

template<typename T>
bool Hamming<T>::create_window(int nsamples) {

	this->nsamples_ = nsamples;

	this->window_ = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(this->nsamples_);

	for(auto i = 0; i<this->nsamples_; i++)
		this->window_(i)   = (54.0 - 46.0*cos((2.0 * M_PI * i)/(this->nsamples_-1))) / 100.0;
	
	
	this->wnorm_ = (this->window_.array().pow(2)).sum() / this->window_.size();

	return true;
}

template<typename T>
DynamicMatrix<T> Hamming<T>::apply(const DynamicMatrix<T>& in) {

	if(this->is_window_set_ == false) {
		this->is_window_set_ = this->create_window(in.rows());
		
		if(this->is_window_set_ == false)
			throw std::runtime_error("[" + this->name() + "] - First apply: cannot create the window");
		else	
			ROS_WARN("[%s] First apply: the window is set", this->name().c_str());
	}
	
	return in.array() * this->window_.replicate(1, in.cols()).array();
}

}


#endif
