#ifndef ROSNEURO_FILTERS_DC_HPP
#define ROSNEURO_FILTERS_DC_HPP

#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"
  
namespace rosneuro {

template <typename T>
class Dc : public Filter<T> {
	public:
		Dc(void);
		~Dc(void) {};

		bool configure(void);
		DynamicMatrix<T> apply(const DynamicMatrix<T>& in);

};

template<typename T>
Dc<T>::Dc(void) {
	this->name_ = "dc";
}

template<typename T>
bool Dc<T>::configure(void) {
	return true;
}

template<typename T>
DynamicMatrix<T> Dc<T>::apply(const DynamicMatrix<T>& in) {
	return in - (in.colwise().mean()).replicate(in.rows(), 1);
}


}

#endif
