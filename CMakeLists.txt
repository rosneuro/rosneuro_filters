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
			 pluginlib)

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
  ${PROJECT_NAME}_car 
  ${PROJECT_NAME}_dc
  ${PROJECT_NAME}_laplacian
  ${PROJECT_NAME}_butterworth
  #${PROJECT_NAME}_hann
  ${PROJECT_NAME}_hamming
  ${PROJECT_NAME}_blackman
  ${PROJECT_NAME}_flattop
	
  CATKIN_DEPENDS
  	roscpp
	roslib
	rosconsole
	std_msgs
	pluginlib
  DEPENDS
)


##############################################################################
# Build
##############################################################################

add_definitions(${EIGEN3_DEFINITIONS})

## Plugin

# CAR Spatial filter
add_library(${PROJECT_NAME}_car src/Car.cpp)
target_link_libraries(${PROJECT_NAME}_car ${Eigen3_LIBRARIES} 
										  ${catkin_LIBRARIES}
)

# DC Spatial filter
add_library(${PROJECT_NAME}_dc src/Dc.cpp)
target_link_libraries(${PROJECT_NAME}_dc ${Eigen3_LIBRARIES} 
										 ${catkin_LIBRARIES}
)

# Laplacian Spatial filter
add_library(${PROJECT_NAME}_laplacian src/Laplacian.cpp)
target_link_libraries(${PROJECT_NAME}_laplacian ${Eigen3_LIBRARIES} 
												${catkin_LIBRARIES}
)

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

# Hamming Window
add_library(${PROJECT_NAME}_hamming src/Hamming.cpp)
target_link_libraries(${PROJECT_NAME}_hamming ${Eigen3_LIBRARIES} 
											  ${catkin_LIBRARIES}
)

# Flattop Window
add_library(${PROJECT_NAME}_flattop src/Flattop.cpp)
target_link_libraries(${PROJECT_NAME}_flattop ${Eigen3_LIBRARIES} 
											  ${catkin_LIBRARIES}
)

# Blackman Window
add_library(${PROJECT_NAME}_blackman src/Blackman.cpp)
target_link_libraries(${PROJECT_NAME}_blackman ${Eigen3_LIBRARIES} 
											   ${catkin_LIBRARIES}
)

#################
## Executables ##
#################

add_executable(car_simloop_config test/car_simloop_config.cpp)
target_link_libraries(car_simloop_config ${PROJECT_NAME}_car ${catkin_LIBRARIES}) 

add_executable(dc_simloop_config test/dc_simloop_config.cpp)
target_link_libraries(dc_simloop_config ${PROJECT_NAME}_dc ${catkin_LIBRARIES}) 

add_executable(laplacian_simloop test/laplacian_simloop.cpp)
target_link_libraries(laplacian_simloop ${PROJECT_NAME}_laplacian ${catkin_LIBRARIES}) 

add_executable(laplacian_simloop_config test/laplacian_simloop_config.cpp)
target_link_libraries(laplacian_simloop_config ${PROJECT_NAME}_laplacian ${catkin_LIBRARIES}) 

add_executable(butterworth_simloop test/butterworth_simloop.cpp)
target_link_libraries(butterworth_simloop ${PROJECT_NAME}_butterworth ${catkin_LIBRARIES}) 

add_executable(butterworth_simloop_config test/butterworth_simloop_config.cpp)
target_link_libraries(butterworth_simloop_config ${PROJECT_NAME}_butterworth ${catkin_LIBRARIES}) 

add_executable(butterworth_simloop_chain test/butterworth_simloop_chain.cpp)
target_link_libraries(butterworth_simloop_chain ${PROJECT_NAME}_butterworth ${catkin_LIBRARIES}) 

add_executable(blackman_window test/blackman_window.cpp)
target_link_libraries(blackman_window ${PROJECT_NAME}_blackman ${catkin_LIBRARIES}) 

add_executable(flattop_window test/flattop_window.cpp)
target_link_libraries(flattop_window ${PROJECT_NAME}_flattop ${catkin_LIBRARIES}) 

add_executable(hamming_window test/hamming_window.cpp)
target_link_libraries(hamming_window ${PROJECT_NAME}_hamming ${catkin_LIBRARIES}) 

#################
##   Install   ##
#################
install(TARGETS ${PROJECT_NAME}_car
				${PROJECT_NAME}_dc
				${PROJECT_NAME}_laplacian
				${PROJECT_NAME}_butterworth
				#${PROJECT_NAME}_hann
				${PROJECT_NAME}_hamming
				${PROJECT_NAME}_flattop
				${PROJECT_NAME}_blackman

	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugins_frequencyfilters.xml
			  plugins_spatialfilters.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
