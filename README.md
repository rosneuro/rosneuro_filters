# ROSNeuro Filters package
The package provides a generic interface **rosneuro::Filter\<T\>** to implement filters to be applied to the input data *[samples x channels]*. The assumption is that filter implemented using this interface will not change the dimension of the input data. Different types of filters can be independently developed and dynamically loaded through the interface. Currently, **ROSNeuro** provides the following plugins:

#### Temporal filters:
- [rosneuro_filters_dc](xxxx) (simple filter to remove the DC component from the data)
- [rosneuro_filters_butterworth](xxxx) (butterworth realt-time filter to low- and high-pass the data) 

#### Spatial filters
- [rosneuro_filters_car](xxxx) (Common Average Reference filter to remove the average component from the data)
- [rosneuro_filters_laplacian](xxxx) (filter to apply Laplacian derivation to the data)

#### Windows
- [rosneuro_filters_blackman](xxxx) (Blackman window to input data)
- [rosneuro_filters_flattop](xxxx) (Flattop window to the input data)
- [rosneuro_filters_hamming](xxxx) (Hamming window to the input data)
- [rosneuro_filters_hann](xxxx) (Hann window to the input data)

Furthermore, the package provides a **rosneuro::FilterChain\<T\>** in order to dynamically concatenate differente filters.

## Requirements
rosneuro_filters has been tested with the following configuration:
- **Ubuntu 18.04.05 LTS Bionic Beaver** and **ROS Melodic**
- **Ubuntu 20.04.05 LTS Focal Fossa** and **ROS Noetic**

rosneuro_filters depends on:
- [Eigen library](https://eigen.tuxfamily.org/index.php?title=Main_Page)

## Usage
Once instanciated, a filter plugin creates a filter that can be applied to input data (**Eigen::Matrix\<T, Eigen::Dynamic, Eigen::Dynamic\>**) with the provided type **T**. Each plugin has specific configuration parameters that can be retrived from the nameserver or can be set manually. Once the filter is set, it can be applied through the function member *DynamicMatrix\<T\> apply(const DynamicMatrix\<T\>& in)*. Here a partial example of the application of the filter **rosneuro::Car\<T\>** to random generated data:

```cpp
#include <ros/ros.h>
#include "rosneuro_filters_car/Car.hpp"

int main(int argc, char** argv) {

	ros::init(argc, argv, "car");
	const int nsamples  = 512;
	const int nchannels = 16;
	
	rosneuro::Filter<float>* car = new rosneuro::Car<float>();
	Eigen::MatrixXf data   = Eigen::MatrixXf::Random(nsamples, nchannels);
	
	Eigen::MatrixXf output = car->apply(data);
		
	ROS_INFO("[%s] Car filter succesfully applied", car->name().c_str());

	return 0;
}
```
As noticed, in this case the filter does not need configuration.

## Filter chain
It is possible to concatenate different filters thanks to the class **rosneuro::FilterChain\<T\>**. Here an example of the filter concatenation and the related YAML configuration file:
```cpp
#include <ros/ros.h>
#include <Eigen/Dense>
#include "rosneuro_filters/FilterChain.hpp"

int main(int argc, char** argv) {


	ros::init(argc, argv, "filterchain");
	
	const int nsamples  = 512;
	const int nchannels = 16;

	rosneuro::FilterChain<float> chain("float");

	if(chain.configure("ChainCfgTest") == false) {
		ROS_ERROR("Chain configuration failed");
		return false;
	}
	ROS_INFO("Chain configuration succeeded");

	Eigen::MatrixXf data = Eigen::MatrixXf::Random(nsamples, nchannels);
			
	Eigen::MatrixXf output = chain.apply(data);
		
	std::cout<<"Filter chain succesfully applied"<<std::endl;
	
	return 0;

}
```
```
ChainCfgTest:
    - name: dc
      type: rosneuro_filters/DcFilterFloat
    - name: car
      type: rosneuro_filters/CarFilterFloat
    - name: hann
      type: rosneuro_filters/HannWindowFloat
    - name: flattop
      type: rosneuro_filters/FlattopWindowFloat
```
## How to implement a custom rosneuro::Filter plugin
In the following section, we describe how to implement a custom plugin based on *rosneuro::Filter* class. We assume that the plugin is implemente in a new package named **myfilters_package** and the plugin is named **MyFilter**. Furthermore we assume that the *myfilter_package* is structured as follows:
```
myfilters_package/
	|- include/myfilters_package/MyFilter.hpp
	|- src/
	   |- MyFilter.cpp
	   |- myfilter.cpp
	|- CMakeLists.txt
	|- package.xml
	|- plugin_myfilter.xml
```
The *MyFilter* class derives from the *rosneuro::Filter* base class and it requires to implement the two pure virtual function members *DynamicMatrix\<T\> apply(const DynamicMatrix\<T\>& in)* and *bool configure(void)*. Here an example of the implementation of the class in the *include/myfilters_package/MyFilter.hpp* file:
```cpp
#ifndef ROSNEURO_MYFILTER_HPP
#define ROSNEURO_MYFILTER_HPP

#include "rosneuro_filters/Filter.hpp"

namespace rosneuro {

template<typename T>
class MyFilter : public Filter<T> {

	public:
		MyFilter(void) {};
		~MyFilter(void) {};

		bool configure(void);
		DynamicMatrix<T> apply(const DynamicMatrix<T>& in);

};

template<typename T>
bool MyFilter<T>::configure(void) {
  
  // Possible parameters from the nameserver

	return true;
}

template<typename T>
DynamicMatrix<T> MyFilter<T>::apply(const DynamicMatrix<T>& in) {
	// Specific implementation of MyFilter
}

}

#endif
```
Notice that in the case of *FilterChain* the function member *MyFilter\<T\>::configure(void)* is automatically called inside the method *rosneuro::Filter\<T\>::configure(const std::string& name)*. Therefore, in order to execute the function member *MyFilter\<T\>::configure(void)* is required to call the function *bool configure(const std::string& name)* in the executable with argument the name of the YAML structure.

In *src/MyFilter.cpp*, we just add the plugin macros:
```cpp
#include "rosneuro_buffers/MyFilter.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS(rosneuro::MyFilter<int>,    rosneuro::Filter<int>)
PLUGINLIB_EXPORT_CLASS(rosneuro::MyFilter<float>,  rosneuro::Filter<float>)
PLUGINLIB_EXPORT_CLASS(rosneuro::MyFilter<double>, rosneuro::Filter<double>)
```
In the *CMakeLists.txt* we need to provide the rules to compile the plugin library:
```
cmake_minimum_required(VERSION 2.8.3)
project(myfilters_package)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 roslib
			 std_msgs
			 pluginlib)
find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}_myfilter
  CATKIN_DEPENDS
    roslib
  	roscpp
	std_msgs
	pluginlib
  DEPENDS
)


###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME}_myfilter src/MyFilter.cpp)
target_link_libraries(${PROJECT_NAME}_myfilter ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_myfilter ${catkin_EXPORTED_TARGETS})

#################
## Executables ##
#################

add_executable(myfilter src/myfilter.cpp)
target_link_libraries(myfilter ${PROJECT_NAME}_myfilter)
```
In the *package.xml* we need to add the dependency to the pluginlib package and to export our myfilter plugin:
```xml
<?xml version="1.0"?>
<package format="2">
  <name>myfilters_package</name>
  <version>0.0.1</version>
  <description>My filters package</description>
  <author email="luca.tonin@unipd.it">Luca Tonin</author>
  <maintainer email="luca.tonin@unipd.it">Luca Tonin</maintainer>

  <license>GPLv3</license>

  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  <depend>message_generation</depend>

  <exec_depend>message_runtime</exec_depend>
  <depend>eigen</depend>
  <depend>roslib</depend>
  <depend>rosconsole</depend>
  <build_depend>pluginlib</build_depend>
  <exec_depend>pluginlib</exec_depend>

  <export>
    <rosneuro_buffers plugin="${prefix}/plugin_myfilter.xml" />
  </export>
  

</package>
```
Finally, we need to add the description of the plugin in the *plugin_myfilter.xml* file:
```xml
<class_libraries>
  <library path="lib/libmyfilters_package">
    <class name="myfilter_package/MyFilterDouble" type="rosneuro::MyFilter<double>"
        base_class_type="rosneuro::Filter<double>">
      <description>MyFilter with doubles</description>
    </class>
    
	<class name="myfilters_package/MyFilterFloat" type="rosneuro::MyFilter<float>"
        base_class_type="rosneuro::Filter<float>">
      <description>MyFilter with floats</description>
    </class>
    
    <class name="myfilters_package/MyFilterInt" type="rosneuro::MyFilter<int>"
        base_class_type="rosneuro::Filter<int>">
      <description>MyFilter with ints</description>
    </class>
  </library>
</class_libraries> 
```
