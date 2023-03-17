cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_filters)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(ROSNEURO_DATA_MIN_VERSION "1.0.0")

##############################################################################
# Find dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 roslib
			 rosconsole
			 std_msgs
			 pluginlib
			 rosneuro_buffers)

find_package(Eigen3 REQUIRED)
find_library(RTF_LIBRARIES rtfilter)
find_path(RTF_INCLUDE_DIRS rtfilter.h)

find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIRS}
					${RTF_INCLUDE_DIRS}
					include)

##############################################################################
# Define package
##############################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  #${PROJECT_NAME}_car 
  #${PROJECT_NAME}_dc
  #${PROJECT_NAME}_laplacian
	${PROJECT_NAME}_butterworth
	#${PROJECT_NAME}_hann
	#${PROJECT_NAME}_hamming
	#${PROJECT_NAME}_blackman
	#${PROJECT_NAME}_flattop
  CATKIN_DEPENDS
  	roscpp
	roslib
	rosconsole
	std_msgs
	pluginlib
	rosneuro_buffers
  DEPENDS
)


##############################################################################
# Build
##############################################################################

add_definitions(${EIGEN3_DEFINITIONS})

## Plugin

# CAR Spatial filter
#add_library(${PROJECT_NAME}_car src/Car.cpp)
#target_link_libraries(${PROJECT_NAME}_car ${Eigen3_LIBRARIES} 
#										  ${catkin_LIBRARIES}
#)
#
## DC Spatial filter
#add_library(${PROJECT_NAME}_dc src/Dc.cpp)
#target_link_libraries(${PROJECT_NAME}_dc ${Eigen3_LIBRARIES} 
#										 ${catkin_LIBRARIES}
#)
#
## Laplacian Spatial filter
#add_library(${PROJECT_NAME}_laplacian src/Laplacian.cpp)
#target_link_libraries(${PROJECT_NAME}_laplacian ${Eigen3_LIBRARIES} 
#												${catkin_LIBRARIES}
#)

# Butterworth frequency filter
add_library(${PROJECT_NAME}_butterworth src/Butterworth.cpp)
target_link_libraries(${PROJECT_NAME}_butterworth ${Eigen3_LIBRARIES} 
												  ${RTF_LIBRARIES} 
												  ${catkin_LIBRARIES}
)

## Hann Window
#add_library(${PROJECT_NAME}_hann src/Hann.cpp)
#target_link_libraries(${PROJECT_NAME}_hann ${Eigen3_LIBRARIES} 
#										   ${catkin_LIBRARIES}
#)
#
## Hamming Window
#add_library(${PROJECT_NAME}_hamming src/Hamming.cpp)
#target_link_libraries(${PROJECT_NAME}_hamming ${Eigen3_LIBRARIES} 
#											  ${catkin_LIBRARIES}
#)
#
## Flattop Window
#add_library(${PROJECT_NAME}_flattop src/Flattop.cpp)
#target_link_libraries(${PROJECT_NAME}_flattop ${Eigen3_LIBRARIES} 
#											  ${catkin_LIBRARIES}
#)
#
## Blackman Window
#add_library(${PROJECT_NAME}_blackman src/Blackman.cpp)
#target_link_libraries(${PROJECT_NAME}_blackman ${Eigen3_LIBRARIES} 
#											   ${catkin_LIBRARIES}
#)

#################
## Executables ##
#################

#add_executable(test_car test/test_car.cpp)
#target_link_libraries(test_car ${PROJECT_NAME}_car ${catkin_LIBRARIES})
#
#add_executable(test_dc test/test_dc.cpp)
#target_link_libraries(test_dc ${PROJECT_NAME}_dc ${catkin_LIBRARIES})
#
#add_executable(test_hann test/test_hann.cpp)
#target_link_libraries(test_hann ${PROJECT_NAME}_hann ${catkin_LIBRARIES})
#
#add_executable(test_hamming test/test_hamming.cpp)
#target_link_libraries(test_hamming ${PROJECT_NAME}_hamming ${catkin_LIBRARIES})
#
#add_executable(test_flattop test/test_flattop.cpp)
#target_link_libraries(test_flattop ${PROJECT_NAME}_flattop ${catkin_LIBRARIES})
#
#add_executable(test_blackman test/test_blackman.cpp)
#target_link_libraries(test_blackman ${PROJECT_NAME}_blackman ${catkin_LIBRARIES})
#
#add_executable(test_chain test/test_chain.cpp)
#target_link_libraries(test_chain ${PROJECT_NAME}_car ${PROJECT_NAME}_dc ${catkin_LIBRARIES}) 

#add_executable(test_laplacian test/test_laplacian.cpp)
#target_link_libraries(test_laplacian ${PROJECT_NAME}_laplacian ${catkin_LIBRARIES}) 

#add_executable(test_butterworth test/test_butterworth.cpp)
#target_link_libraries(test_butterworth ${PROJECT_NAME}_butterworth ${catkin_LIBRARIES}) 

#add_executable(test_bandpass_filter test/test_bandpass_filter.cpp)
#target_link_libraries(test_bandpass_filter ${PROJECT_NAME}_butterworth ${catkin_LIBRARIES}) 

#add_executable(test_bandpass_filter_eigen test/test_bandpass_filter_eigen.cpp)
#target_link_libraries(test_bandpass_filter_eigen ${PROJECT_NAME}_butterworth ${catkin_LIBRARIES}) 

add_executable(butterworth_simloop test/butterworth_simloop.cpp)
target_link_libraries(butterworth_simloop ${PROJECT_NAME}_butterworth ${catkin_LIBRARIES}) 

#add_executable(test_rtfilter test/test_rtfilter.cpp)
#target_link_libraries(test_rtfilter ${PROJECT_NAME}_butterworth ${catkin_LIBRARIES}) 

#add_executable(test test/test.cpp)
#target_link_libraries(test ${PROJECT_NAME}_car ${PROJECT_NAME}_dc ${catkin_LIBRARIES}) 

#################
##   Install   ##
#################
install(TARGETS #${PROJECT_NAME}_car
				#${PROJECT_NAME}_dc
				#${PROJECT_NAME}_laplacian
				${PROJECT_NAME}_butterworth
				#${PROJECT_NAME}_hann
				#${PROJECT_NAME}_hamming
				#${PROJECT_NAME}_flattop
				#${PROJECT_NAME}_blackman
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugins_frequencyfilters.xml
			  plugins_spatialfilters.xml
			  plugins_windows.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
