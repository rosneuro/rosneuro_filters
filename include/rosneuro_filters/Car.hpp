#ifndef ROSNEURO_FILTERS_CAR_HPP
#define ROSNEURO_FILTERS_CAR_HPP

#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"

namespace rosneuro {

template <typename T>
class Car : public Filter<T> {
	public:
		Car(void);
		~Car(void) {};

		bool configure(void);
		DynamicMatrix<T> apply(const DynamicMatrix<T>& in);

};

template<typename T>
Car<T>::Car(void) {
	this->name_ = "car";
}

template<typename T>
bool Car<T>::configure(void) {
	return true;
}

template<typename T>
DynamicMatrix<T> Car<T>::apply(const DynamicMatrix<T>& in) {
	return in - (in.rowwise().mean()).replicate(1, in.cols());
}


}

#endif
