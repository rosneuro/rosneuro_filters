#ifndef ROSNEURO_FILTERS_FLATTOP_HPP
#define ROSNEURO_FILTERS_FLATTOP_HPP

#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"
  
namespace rosneuro {

template <typename T>
class Flattop : public Filter<T> {
	
	public:
		Flattop(void);
		~Flattop(void) {};

		bool configure(void);
		DynamicMatrix<T> apply(const DynamicMatrix<T>& in);

	private:
		bool create_window(int nsamples);

	private:
		int nsamples_;
		bool is_window_set_;
		Eigen::Matrix<T, Eigen::Dynamic, 1> window_;
		T wnorm_;

		static constexpr double A0 = 0.215578950;
    	static constexpr double A1 = 0.416631580;
    	static constexpr double A2 = 0.277263158;
    	static constexpr double A3 = 0.083578947;
    	static constexpr double A4 = 0.006947368;		
		
};

template<typename T>
Flattop<T>::Flattop(void) {
	this->name_ 	     = "flattop";
	this->is_window_set_ = false;
}

template<typename T>
bool Flattop<T>::configure(void) {
	return true;
}


template<typename T>
DynamicMatrix<T> Flattop<T>::apply(const DynamicMatrix<T>& in) {

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
bool Flattop<T>::create_window(int nsamples) {
	
	this->nsamples_ = nsamples;
	
	this->window_ = Eigen::Matrix<T, Eigen::Dynamic, 1>::Zero(this->nsamples_);

	for(auto i = 0; i<this->nsamples_; i++)
		this->window_(i)   = this->A0 - this->A1*cos((2.0 * M_PI * i)/(this->nsamples_-1)) + this->A2*cos((4.0 * M_PI * i)/(this->nsamples_-1)) - this->A3*cos((6.0 * M_PI * i)/(this->nsamples_-1)) + this->A4*cos((8.0 * M_PI * i)/(this->nsamples_-1));
	
	this->wnorm_ = (this->window_.array().pow(2)).sum() / this->window_.size();

	return true;
}

}

#endif
