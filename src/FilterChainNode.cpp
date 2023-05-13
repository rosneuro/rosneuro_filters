#include "rosneuro_filters/FilterChainNode.h"

namespace rosneuro {

FilterChainNode::FilterChainNode(void) : p_nh_("~") {

	this->sub_ = this->nh_.subscribe("/neurodata", 1, &FilterChainNode::on_received_neurodata, this);
	this->pub_ = this->nh_.advertise<rosneuro_msgs::NeuroFrame>("/neurodata_filtered", 1);
}

FilterChainNode::~FilterChainNode(void) {
}

bool FilterChainNode::configure(void) {

	this->p_nh_.param<std::string>("configname", this->cfg_name_, "filterchain");

	if(this->chain_.configure(this->cfg_name_) == false) {
		ROS_ERROR("[FilterChain] FilterChain configuration failed");
		return false;
	}
		
	ROS_INFO("[FilterChain] FilterChain has been configured");

	return true;
}

void FilterChainNode::run(void) {

	ros::Rate r(512);

	while(ros::ok()) {

		ros::spinOnce();
		r.sleep();
	}

}

void FilterChainNode::on_received_neurodata(const rosneuro_msgs::NeuroFrame& msg) {

	float* ptr_in;
	rosneuro::DynamicMatrix<float> data_in_T;
	rosneuro::DynamicMatrix<float> data_out_T;
	rosneuro::DynamicMatrix<float> data_out;

	unsigned int nsamples   = msg.eeg.info.nsamples;
	unsigned int nchannels  = msg.eeg.info.nchannels;

	std::vector<float> neuroeeg(nsamples*nchannels);

	// Getting pointer to the input data message
	ptr_in = const_cast<float*>(msg.eeg.data.data());

	// Re-map raw pointer to eigen matrix.
	// Remember input data is stored as [channels x samples]
	rosneuro::DynamicMatrix<float> data_in = Eigen::Map<rosneuro::DynamicMatrix<float>>(ptr_in, nchannels, nsamples);

	// Transpose input data to [samples x channels]
	data_in_T = data_in.transpose();

	// Apply filter chain
	data_out_T = this->chain_.apply(data_in_T);

	// Transpose again data out to [channels x samples]
	data_out = data_out_T.transpose();

	// Re-map eigen matrix to std::vector
	Eigen::Map<rosneuro::DynamicMatrix<float>>(neuroeeg.data(), nchannels, nsamples) = data_out;

	// Setting message
	this->neuromsg_.header.stamp = ros::Time::now();
	this->neuromsg_.sr  = msg.sr;
	this->neuromsg_.exg = msg.exg;
	this->neuromsg_.tri = msg.tri;
	this->neuromsg_.eeg.info = msg.eeg.info;
	this->neuromsg_.eeg.data = neuroeeg;

	// Publishing the message
	this->pub_.publish(this->neuromsg_);
}



}
