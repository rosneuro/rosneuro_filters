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
        rosneuro::DynamicMatrix<float> data_in_T, data_out_T, data_out;

        unsigned int nsamples   = msg.eeg.info.nsamples;
        unsigned int nchannels  = msg.eeg.info.nchannels;

        std::vector<float> neuroeeg(nsamples*nchannels);

        float* ptr_in = const_cast<float*>(msg.eeg.data.data());

        rosneuro::DynamicMatrix<float> data_in = Eigen::Map<rosneuro::DynamicMatrix<float>>(ptr_in, nchannels, nsamples);
        data_in_T = data_in.transpose();
        data_out_T = this->chain_.apply(data_in_T);
        data_out = data_out_T.transpose();
        Eigen::Map<rosneuro::DynamicMatrix<float>>(neuroeeg.data(), nchannels, nsamples) = data_out;

        this->neuromsg_.header.stamp = ros::Time::now();
        this->neuromsg_.sr  = msg.sr;
        this->neuromsg_.exg = msg.exg;
        this->neuromsg_.tri = msg.tri;
        this->neuromsg_.eeg.info = msg.eeg.info;
        this->neuromsg_.eeg.data = neuroeeg;

        this->pub_.publish(this->neuromsg_);
    }
}
