#ifndef ROSNEURO_FILTERS_LAPLACIAN_HPP
#define ROSNEURO_FILTERS_LAPLACIAN_HPP

#include <regex>
#include <Eigen/Dense>
#include "rosneuro_filters/Filter.hpp"

namespace rosneuro {

template <typename T>
class Laplacian : public Filter<T> {
	public:
		Laplacian(void);
		~Laplacian(void) {};

		bool configure(void);
		DynamicMatrix<T> apply(const DynamicMatrix<T>& in);

	private:
		bool load_layout(const std::string slayout);
		bool has_duplicate(const std::string slayout);
		bool find_channel(unsigned int channel, unsigned int& rId, unsigned int& cId); 
		bool create_mask(void);
		std::vector<int> get_neightbours(unsigned int rId, unsigned int cId);
	
	private:
		unsigned int nchannels_;
		DynamicMatrix<int> layout_;
		DynamicMatrix<T> mask_;
};

template<typename T>
Laplacian<T>::Laplacian(void) {
	this->name_ = "laplacian";
}


template<typename T>
bool Laplacian<T>::configure(void) {

	bool retcod = false;
	std::string layout_str;
	
	if (!Filter<T>::getParam(std::string("layout"), layout_str)) {
    	ROS_ERROR("[%s] Cannot find param layout", this->name().c_str());
		return false;
	}
	

	if(this->has_duplicate(layout_str)) {
    	ROS_ERROR("[%s] The provided layout has duplicated indexes", this->name().c_str());
		return false;
	}

	if(this->load_layout(layout_str) == false) {
		ROS_ERROR("[%s] The provided layout is wrongly formatted", this->name().c_str());
		return false;
	}
	
	if (!Filter<T>::getParam(std::string("nchannels"), this->nchannels_)) {
		this->nchannels_ = this->layout_.maxCoeff();
    	ROS_WARN("[%s] Number of channels not provided: assuming that the number of channels " 
				 "corresponds to the highest index in the provided layout (%d)", 
				 this->name().c_str(), this->nchannels_);
		retcod = true;
	}

	if(this->create_mask() == false) {
		ROS_ERROR("[%s] Cannot create laplacian mask", this->name().c_str());
		return false;
	}

	return retcod;
}

template<typename T>
bool Laplacian<T>::create_mask(void) {

	this->mask_ = DynamicMatrix<T>::Zero(this->nchannels_, this->nchannels_);
	for(auto chIdx=1; chIdx<=this->nchannels_; chIdx++) {

		unsigned int crowId;
		unsigned int ccolId;
		std::vector<int> neightbours;

		if(find_channel(chIdx, crowId, ccolId)) {

			neightbours = get_neightbours(crowId, ccolId);
		
			this->mask_(chIdx-1, chIdx-1) = 1;
			for(auto it=neightbours.begin(); it!=neightbours.end(); ++it)
				this->mask_((*it)-1, chIdx-1) = -1./neightbours.size();
		}
	}

	return true;

}

template<typename T>
bool Laplacian<T>::find_channel(unsigned int channel, unsigned int& rId, unsigned int& cId) {

	bool retval = false;
	unsigned int nrows = this->layout_.rows();
	unsigned int ncols = this->layout_.cols();
	
	for(auto i=0; i<nrows; i++) {
		for (auto j=0; j<ncols; j++) {
			if(this->layout_(i, j) == channel) {
				rId = i;
				cId = j;
				retval = true;
				break;
			}
		}
	}
	return retval;
}

template<typename T>
std::vector<int> Laplacian<T>::get_neightbours(unsigned int rId, unsigned int cId) {

	std::vector<int> neightbours;
	unsigned int nrows = this->layout_.rows();
	unsigned int ncols = this->layout_.cols();

	// Left neightbour
	if(cId > 0) {
		if(this->layout_(rId, cId - 1) != 0)
			neightbours.push_back(this->layout_(rId, cId - 1));
	}
	
	// Right neightbour
	if(cId < ncols-1) {
		if(this->layout_(rId, cId + 1) != 0)
			neightbours.push_back(this->layout_(rId, cId + 1));
	}

	// Up neightbour
	if(rId > 0) {
		if(this->layout_(rId - 1, cId) != 0)
			neightbours.push_back(this->layout_(rId - 1, cId));
	}
	
	// Down neightbour
	if(rId < nrows-1) {
		if(this->layout_(rId + 1, cId) != 0)
			neightbours.push_back(this->layout_(rId + 1, cId));
	}

	return neightbours;
}

template<typename T>
bool Laplacian<T>::has_duplicate(const std::string slayout) {

	const std::regex pattern("\\b([1-9]+)(?:\\W+\\1\\b)+", std::regex_constants::icase);
	
	auto it = std::sregex_iterator(slayout.begin(), slayout.end(), pattern);
	if(it == std::sregex_iterator()) {
		return false;
	}
	
	return true;
}

template<typename T>
bool Laplacian<T>::load_layout(const std::string slayout) {
	unsigned int nrows;
	unsigned int ncols;
	
	std::vector<std::vector<int>> vlayout;

	std::stringstream ss(slayout);
	std::string srow;

	while(getline(ss, srow, ';')) {
		std::stringstream iss(srow);
		int index;
		std::vector<int> vrow;
		while(iss >> index) {
			vrow.push_back(index);
		}
		vlayout.push_back(vrow);
	}

	nrows = vlayout.size();
	ncols = vlayout.at(0).size();
	
	for(auto it=vlayout.begin(); it!=vlayout.end(); ++it) {
		if( (*it).size() != ncols )
			return false;
	}



	this->layout_ = DynamicMatrix<int>::Zero(nrows, ncols);

	for (auto i=0; i<this->layout_.rows(); i++) {
		for(auto j=0; j<this->layout_.cols(); j++) {
			this->layout_(i, j) = vlayout.at(i).at(j);
		}
	}

	return true;
}

template<typename T>
DynamicMatrix<T> Laplacian<T>::apply(const DynamicMatrix<T>& in) {
	return in * this->mask_;
}


}

#endif
