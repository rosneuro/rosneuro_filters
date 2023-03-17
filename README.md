# ROSNeuro Filters package
The package provides a generic interface to implement filters to be applied to the data. Different types of filters can be independently developed and dynamically loaded through the interface. Currently, the package provides the following plugins:

#### Temporal filters
- **rosneuro::Dc\<T\>** filter (to remove the DC component)
- **rosneuro::Butterworth\<T\>** butterworth filter for low-pass and high-pass 

#### Spatial filters
- **rosneuro::Car\<T\>** filter (to apply a Common Average Reference filter)
- **rosneuro::Laplacian\<T\>** filter (to apply Laplacian derivation to the)

Furthermore, the package provides a **rosneuro::FilterChain\<T\>** in order to dynamically concatenate differente filters.

## Requirements
rosneuro_filters has been tested with the following configuration:
- **Ubuntu 18.04.05 LTS Bionic Beaver** and **ROS Melodic**
- **Ubuntu 20.04.02 LTS Focal Fossa** and **ROS Noetic**

rosneuro_filters depends on:
- [Eigen library](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [librtfilter-dev](https://neuro.debian.net/pkgs/librtfilter-dev.html)

## TODO 

%## Usage
%Once instanciated, a filter plugin creates a filter that can be applied to **NeuroData** with the provided type **T**. Each plugin has specific configuration parameters retrived from the nameserver. Once the configuration is succesfully done, the filter is applied through the function member *bool apply(const NeuroData/<T/>& data_in, NeuroData/<T/>& data_out)*. Here a partial example of the application of the filter **rosneuro::Car\<T\>** to random generated data:
%
%```cpp
%#include <ros/ros.h>
%#include "rosneuro_data/NeuroData.hpp"
%#include "rosneuro_filters/Car.hpp"
%
%int main(int argc, char** argv) {
%
%	ros::init(argc, argv, "car");
%	
%	rosneuro::Filter<float>* car = new rosneuro::Car<float>();
%	if(car->configure("CarCfgTest") == false) {
%		ROS_ERROR("Car filter configuration failed");
%		return false;
%	}
%	ROS_INFO("Car filter configuration succeeded");
%
%	rosneuro::NeuroData<float> in(2, 3, "EEG");
%	rosneuro::NeuroData<float> out(2, 3, "EEG");
%			
%	Eigen::MatrixXf data = Eigen::MatrixXf::Random(2, 3);
%	std::cout<<data<<std::endl;
%	std::copy(data.data(), data.data() + data.size(), in.data());
%
%	if(car->apply(in, out) == false) {
%		std::cout<<"Car filter failed"<<std::endl;
%		ros::shutdown();
%		return -1;
%	}
%		
%	std::cout<<"Car filter succesfully applied"<<std::endl;
%	Eigen::Map<Eigen::MatrixXf> eout(out.data(), 2, 3);
%
%	std::cout<<eout<<std::endl;
%
%	return 0;
%}
%```
%As noticed, the filter is configured through the function:
%```cpp
%if(car->configure("CarCfgTest") == false) {
%  ROS_ERROR("Car filter configuration failed");
%  return false;
%}
%```
%that takes as argument the name provided in the YAML configuration file:
%```
%CarCfgTest:
%  name: car
%  type: CarFilterFloat
%```
%## Filter plugins
%The package provides different filter types. Herein, the list of the filters with the required parameters (as YAML file):
%
%### Temporal filters
%**rosneuro::Dc\<T\>** filter
%It applies DC removal from the data by substracting the mean value from each channel. The mean value is computed over all the provided samples.
%```
%DcCfgTest:
%  name: dc
%  type: DcFilterFloat
%```
%*No additional parameters, but name and type*
%
%### Spatial filters
%**rosneuro::Car\<T\>** filter
%It applies the Common Average Reference to the data. For each provided sample, it substracts the mean value computed across all the channel.
%```
%CarCfgTest:
%  name: car
%  type: CarFilterFloat
%```
%*No additional parameters, but name and type*
%
%- **rosneuro::Laplacian\<T\>** filter
%It applies the Laplacian derivation to the data. For each provided sample and for each channel, it substracts the averaged value computed across the neighbour channels. Neighbours are identified with a cross policy:
%```
%         N3
%         ||
% N1 == Channel == N2
%         ||
%         N4
%```
%It is **required** to provide the channel layout as parameter of the filter as well as the total number of channel in the *NeuroData*. The channel layout is defined as a matrix with the index of the channels and 0 in the other positions:
%```
%LaplacianCfgTest:
%  name: laplacian
%  type: LaplacianWindowFloat
%  params: 
%    nchannels: 16
%    layout: " 0  0  1  0  0; 
%              2  3  4  5  6;
%              7  8  9 10 11; 
%             12 13 14 15 16"
%```
%
%### Windows
%Each window filter applies a specific window to the provided data. The current available windows are:
%- **rosneuro::Blackman\<T\>** window 
%- **rosneuro::Flattop\<T\>** window
%- **rosneuro::Hamming\<T\>** window
%- **rosneuro::Hann\<T\>** window
%
%Each window filter has a mandatory parameter that specifies the size of the window (number of samples).
%```
%BlackmanCfgTest:
%  name: blackman
%  type: BlackmanWindowFloat
%  params: {nsamples: 4}
%```
%```
%FlattopCfgTest:
%  name: flattop
%  type: FlattopWindowFloat
%  params: {nsamples: 4}
%```
%```
%HammingCfgTest:
%  name: hamming
%  type: HammingWindowFloat
%  params: {nsamples: 4}
%```
%```
%HannCfgTest:
%  name: hann
%  type: hannWindowFloat
%  params: {nsamples: 4}
%```
%
%## Filter chain
%It is possible to concatenate different filters thanks to the class **rosneuro::FilterChain**. Here an example of the filter concatenation and the related YAML configuration file:
%```cpp
%#include <ros/ros.h>
%#include <ros/console.h>
%#include <Eigen/Dense>
%#include "rosneuro_data/NeuroData.hpp"
%#include "rosneuro_filters/FilterChain.hpp"
%
%int main(int argc, char** argv) {
%
%
%	ros::init(argc, argv, "chain");
%	ros::NodeHandle nh;
%
%	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
%	   ros::console::notifyLoggerLevelsChanged();
%	}
%
%	rosneuro::FilterChain<float> chain("float");
%
%	if(chain.configure("ChainCfgTest") == false) {
%		ROS_ERROR("Chain configuration failed");
%		return false;
%	}
%	ROS_INFO("Chain configuration succeeded");
%
%	rosneuro::NeuroData<float> in(4, 3, "EEG");
%	rosneuro::NeuroData<float> out(4, 3, "EEG");
%			
%	Eigen::MatrixXf data = Eigen::MatrixXf::Random(4, 3);
%	std::cout<<data<<std::endl;
%	std::copy(data.data(), data.data() + data.size(), in.data());
%
%	if(chain.apply(in, out) == false) {
%		ROS_INFO("Chain filter failed");
%		return -1;
%	}
%		
%	std::cout<<"Dc filter succesfully applied"<<std::endl;
%	Eigen::Map<Eigen::MatrixXf> eout(out.data(), 4, 3);
%
%	std::cout<<eout<<std::endl;
%	
%	return 0;
%
%}
%```
%```
%ChainCfgTest:
%    - name: dc
%      type: rosneuro_filters/DcFilterFloat
%    - name: car
%      type: rosneuro_filters/CarFilterFloat
%    - name: hann
%      type: rosneuro_filters/HannWindowFloat
%      params: {nsamples: 4}
%    - name: flattop
%      type: rosneuro_filters/FlattopWindowFloat
%      params: {nsamples: 4}
%```
%
%## How to implement a custom rosneuro::Filter plugin
%In the following section, we describe how to implement a custom plugin based on *rosneuro::Filter* class. We assume that the plugin is implemente in a new package named **myfilters_package** and the plugin is named **MyFilter**. Furthermore we assume that the *myfilter_package* is structured as follows:
%```
%myfilters_package/
%	|- include/myfilters_package/MyFilter.hpp
%	|- src/
%	   |- MyFilter.cpp
%	   |- myfilter.cpp
%	|- CMakeLists.txt
%	|- package.xml
%	|- plugin_myfilter.xml
%```
%The *MyFilter* class derives from the *rosneuro::Filter* base class and it requires to implement the two pure virtual function members *bool apply(const NeuroData\<T\>& data_in, NeuroData\<T\>& data_out)* and *bool configure(void)*. Here an example of the implementation of the class in the *include/myfilters_package/MyFilter.hpp* file:
%```cpp
%#ifndef ROSNEURO_MYFILTER_HPP
%#define ROSNEURO_MYFILTER_HPP
%
%#include "rosneuro_filters/Filter.hpp"
%
%namespace rosneuro {
%
%template<typename T>
%class MyFilter : public Filter<T> {
%
%	public:
%		MyFilter(void) {};
%		~MyFilter(void) {};
%
%		bool configure(void);
%		bool apply(const NeuroData<T>& data_in, NeuroData<T>& data_out);
%
%};
%
%template<typename T>
%bool MyFilter<T>::configure(void) {
%  
%  // Possible parameters from the nameserver
%
%	return true;
%}
%
%template<typename T>
%bool MyFilter<T>::apply(const NeuroData<T>& data_in, NeuroData<T>& data_out){
%
%	// Specific implementation of MyFilter
%}
%
%}
%
%#endif
%```
%Notice that the function member *MyFilter/<T/>::configure(void)* is automatically called inside the method *rosneuro::Filter/<T/>::configure(const std::string& name)*. Therefore, in order to execute the function member *MyFilter/<T/>::configure(void)* is required to call the function *bool configure(const std::string& name)* in the executable with argument the name of the YAML structure.
%
%In *src/MyFilter.cpp*, we just add the plugin macros:
%```cpp
%#include "rosneuro_buffers/MyFilter.hpp"
%#include "pluginlib/class_list_macros.h"
%
%PLUGINLIB_EXPORT_CLASS(rosneuro::MyFilter<int>,    rosneuro::Filter<int>)
%PLUGINLIB_EXPORT_CLASS(rosneuro::MyFilter<float>,  rosneuro::Filter<float>)
%PLUGINLIB_EXPORT_CLASS(rosneuro::MyFilter<double>, rosneuro::Filter<double>)
%```
%In the *CMakeLists.txt* we need to provide the rules to compile the plugin library:
%```
%cmake_minimum_required(VERSION 2.8.3)
%project(myfilters_package)
%
%set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
%
%find_package(catkin REQUIRED COMPONENTS 
%			 roscpp 
%			 roslib
%			 std_msgs
%			 pluginlib
%			 rosneuro_data
%			 rosneuro_msgs)
%find_package(PkgConfig)
%
%SET(CMAKE_BUILD_TYPE RelWithDebInfo)
%
%catkin_package(
%  INCLUDE_DIRS 
%	include
%  LIBRARIES 
%	${PROJECT_NAME}_myfilter
%  CATKIN_DEPENDS
%    roslib
%  	roscpp
%	std_msgs
%	pluginlib
%	rosneuro_data
%	rosneuro_msgs
%  DEPENDS
%)
%
%
%###########
%## Build ##
%###########
%
%include_directories(${catkin_INCLUDE_DIRS} include)
%
%add_library(${PROJECT_NAME}_myfilter src/MyFilter.cpp)
%target_link_libraries(${PROJECT_NAME}_myfilter ${catkin_LIBRARIES})
%add_dependencies(${PROJECT_NAME}_myfilter ${catkin_EXPORTED_TARGETS})
%
%#################
%## Executables ##
%#################
%
%add_executable(myfilter src/myfilter.cpp)
%target_link_libraries(myfilter ${PROJECT_NAME}_myfilter)
%```
%In the *package.xml* we need to add the dependency to the pluginlib package and to export our myfilter plugin:
%```xml
%<?xml version="1.0"?>
%<package format="2">
%  <name>myfilters_package</name>
%  <version>0.0.1</version>
%  <description>My filters package</description>
%  <author email="luca.tonin@dei.unipd.it">Luca Tonin</author>
%  <maintainer email="luca.tonin@dei.unipd.it">Luca Tonin</maintainer>
%
%  <license>GPLv3</license>
%
%  <buildtool_depend>catkin</buildtool_depend>
%  <depend>roscpp</depend>
%  <depend>std_msgs</depend>
%  <depend>message_generation</depend>
%
%  <depend>rosneuro_data</depend>
%  <depend>rosneuro_msgs</depend>
%  <exec_depend>message_runtime</exec_depend>
%  <depend>eigen</depend>
%  <depend>roslib</depend>
%  <depend>rosconsole</depend>
%  <build_depend>pluginlib</build_depend>
%  <exec_depend>pluginlib</exec_depend>
%
%  <export>
%    <rosneuro_buffers plugin="${prefix}/plugin_myfilter.xml" />
%  </export>
%  
%
%</package>
%```
%Finally, we need to add the description of the plugin in the *plugin_myfilter.xml* file:
%```xml
%<class_libraries>
%  <library path="lib/libmyfilters_package">
%    <class name="myfilter_package/MyFilterDouble" type="rosneuro::MyFilter<double>"
%        base_class_type="rosneuro::Filter<double>">
%      <description>MyFilter with doubles</description>
%    </class>
%    
%	<class name="myfilters_package/MyFilterFloat" type="rosneuro::MyFilter<float>"
%        base_class_type="rosneuro::Filter<float>">
%      <description>MyFilter with floats</description>
%    </class>
%    
%    <class name="myfilters_package/MyFilterInt" type="rosneuro::MyFilter<int>"
%        base_class_type="rosneuro::Filter<int>">
%      <description>MyFilter with ints</description>
%    </class>
%  </library>
%</class_libraries> 
%```

