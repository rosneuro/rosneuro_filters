#ifndef ROSNEURO_FILTERS_FILTERCHAIN_HPP_
#define ROSNEURO_FILTERS_FILTERCHAIN_HPP_

#include <vector>
#include <memory>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include "rosneuro_filters/Filter.hpp"
#include <gtest/gtest_prod.h>

namespace rosneuro {
    template<typename T>
    class FilterChain {
        public:
            FilterChain(void);
            ~FilterChain(void);

            bool configure(const std::string param_name, ros::NodeHandle nh = ros::NodeHandle());
            bool configure(XmlRpc::XmlRpcValue& config, const std::string& proc_ns);

            DynamicMatrix<T> apply(const DynamicMatrix<T>& in);
            bool clear(void);
            void dump(void);

        private:
            std::string get_baseclass_name(void);


        private:
            bool configured_;
            std::vector<boost::shared_ptr<rosneuro::Filter<T>>> procs_;
            pluginlib::ClassLoader<rosneuro::Filter<T> >* loader_;
            DynamicMatrix<T> buffer0_;
            DynamicMatrix<T> buffer1_;

            FRIEND_TEST(FilterChainTestSuite, ConstructorTest);
            FRIEND_TEST(FilterChainTestSuite, DestructorTest);
            FRIEND_TEST(FilterChainTestSuite, GetBaseClassName);
            FRIEND_TEST(FilterChainTestSuite, EmptyConfiguration);
            FRIEND_TEST(FilterChainTestSuite, InvalidTypeInConfiguration);
            FRIEND_TEST(FilterChainTestSuite, ValidConfiguration);
            FRIEND_TEST(FilterChainTestSuite, ConfigureTest);
    };

    template<typename T>
    FilterChain<T>::FilterChain(void) {
        this->loader_ = new pluginlib::ClassLoader<rosneuro::Filter<T> >("rosneuro_filters", this->get_baseclass_name());

        std::string lib_string = "";
        std::vector<std::string> libs = this->loader_->getDeclaredClasses();

        for (auto i = 0 ; i < libs.size(); i++) {
           lib_string = lib_string + std::string(", ") + libs[i];
        }

        ROS_DEBUG("In FilterChain ClassLoader found the following libs: %s", lib_string.c_str());
        this->configured_ = false;
    }

    template<typename T>
    FilterChain<T>::~FilterChain(void) {
        this->clear();
        delete this->loader_;
    }

    template<> inline
    std::string FilterChain<float>::get_baseclass_name(void) {
        return "rosneuro::Filter<float>";
    }

    template<> inline
    std::string FilterChain<double>::get_baseclass_name(void) {
        return "rosneuro::Filter<double>";
    }

    template<> inline
    std::string FilterChain<int>::get_baseclass_name(void) {
        return "rosneuro::Filter<int>";
    }

    template<typename T>
    bool FilterChain<T>::clear(void) {
        this->configured_ = false;
        this->procs_.clear();
        return true;
    }

    template<typename T>
    DynamicMatrix<T> FilterChain<T>::apply(const DynamicMatrix<T>& input) {
        DynamicMatrix<T> output;
        unsigned int nprocs = this->procs_.size();

        if(nprocs == 0)
            return input;

        this->buffer0_ = input;
        this->buffer1_ = output;

        for(auto it=this->procs_.begin(); it != this->procs_.end(); ++it) {
            this->buffer1_ = (*it)->apply(this->buffer0_);
            this->buffer0_ = this->buffer1_;
        }

        output = this->buffer1_;
        return output;
    }

    template<typename T>
    void FilterChain<T>::dump(void) {
        for(auto it=this->procs_.begin(); it != this->procs_.end(); ++it)
            (*it)->dump();
    }

    template<typename T>
    bool FilterChain<T>::configure(const std::string param_name, ros::NodeHandle nh) {
        XmlRpc::XmlRpcValue config;
        if(nh.getParam(param_name + "/filters_chain", config)) {
            std::string resolved_name = nh.resolveName(param_name).c_str();
        } else if(!nh.getParam(param_name, config)) {
            ROS_WARN("Could not load the filter chain configuration from parameter \'%s\', are you sure it was pushed to the parameter server? Assuming that you meant to leave it empty.", param_name.c_str());
            configured_ = false;
            return false;
        }
        return this->configure(config, nh.getNamespace());
    }

    template<typename T>
    bool FilterChain<T>::configure(XmlRpc::XmlRpcValue& config, const std::string& filt_ns) {
        if (config.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("%s: The filter chain specification must be a list. but is of of XmlRpcType %d",
                    filt_ns.c_str(), config.getType());
            ROS_ERROR("The xml passed in is formatted as follows:\n %s", config.toXml().c_str());
            return false;
        }

        for (auto i = 0; i < config.size(); ++i) {
            if(config[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_ERROR("%s: Filters must be specified as maps, but they are XmlRpcType:%d",
                          filt_ns.c_str(), config[i].getType());
                return false;
            } else if (!config[i].hasMember("type")) {
                ROS_ERROR("%s: Could not add a filter because no type was given", filt_ns.c_str());
                return false;
            } else if (!config[i].hasMember("name")) {
                ROS_ERROR("%s: Could not add a filter because no name was given", filt_ns.c_str());
                return false;
            } else {
                for (auto j = i + 1; j < config.size(); ++j) {
                    if(config[j].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                        ROS_ERROR("%s: Filters must be specified as maps, but they are XmlRpcType:%d",
                                  filt_ns.c_str(), config[j].getType());
                        return false;
                    }

                    if(!config[j].hasMember("name")
                       ||config[i]["name"].getType() != XmlRpc::XmlRpcValue::TypeString
                       ||config[j]["name"].getType() != XmlRpc::XmlRpcValue::TypeString) {
                            ROS_ERROR("%s: Filters names must be strings, but they are XmlRpcTypes:%d and %d",
                                      filt_ns.c_str(), config[i].getType(), config[j].getType());
                            return false;
                    }

                    std::string namei = config[i]["name"];
                    std::string namej = config[j]["name"];

                    if (namei == namej) {
                        ROS_ERROR("%s: A self_process with the name %s already exists",
                                  filt_ns.c_str(), namei.c_str());
                        return false;
                    }
                }


                if (std::string(config[i]["type"]).find("/") == std::string::npos) {
                    ROS_ERROR("Bad filter type %s. Filter type must be of form <package_name>/<process_name>",
                              std::string(config[i]["type"]).c_str());
                    return false;
                }

                //Make sure the filter chain has a valid type
                std::vector<std::string> libs = this->loader_->getDeclaredClasses();
                bool found = false;
                for (auto it = libs.begin(); it != libs.end(); ++it) {
                    if (*it == std::string(config[i]["type"])) {
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    ROS_ERROR("Couldn't find filter of type %s", std::string(config[i]["type"]).c_str());
                    return false;
                }

            }
        }

        bool result = true;

        for (int i = 0; i < config.size(); ++i) {

            boost::shared_ptr<rosneuro::Filter<T> > p(this->loader_->createInstance(config[i]["type"]));

            if (p.get() == nullptr)
                return false;

            result = result && p.get()->configure(config[i]);
            this->procs_.push_back(p);

            std::string type = config[i]["type"];
            std::string name = config[i]["name"];
            ROS_DEBUG("%s: Configured %s:%s filter at %p\n", filt_ns.c_str(),
                    type.c_str(), name.c_str(),  p.get());
        }

        if (result) {
            this->configured_ = true;
        }

        return result;
    }
}

#endif
