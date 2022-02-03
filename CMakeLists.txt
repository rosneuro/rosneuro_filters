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
			 rosneuro_data
			 rosneuro_msgs)
		 find_package(rosneuro_data ${ROSNEURO_DATA_MIN_VERSION} REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} 
                    ${EIGEN3_INCLUDE_DIRS}
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
	${PROJECT_NAME}_hann
	${PROJECT_NAME}_hamming
	${PROJECT_NAME}_blackman
	${PROJECT_NAME}_flattop
  CATKIN_DEPENDS
  	roscpp
	roslib
	rosconsole
	std_msgs
	pluginlib
	rosneuro_data
	rosneuro_msgs
  DEPENDS
)


##############################################################################
# Build
##############################################################################

add_definitions(${EIGEN3_DEFINITIONS})

## Plugin

# CAR Spatial filter
add_library(${PROJECT_NAME}_car src/Car.cpp)
target_link_libraries(${PROJECT_NAME}_car ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})

# DC Spatial filter
add_library(${PROJECT_NAME}_dc src/Dc.cpp)
target_link_libraries(${PROJECT_NAME}_dc ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})

# Laplacian Spatial filter
add_library(${PROJECT_NAME}_laplacian src/Laplacian.cpp)
target_link_libraries(${PROJECT_NAME}_laplacian ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})

# Hann Window
add_library(${PROJECT_NAME}_hann src/Hann.cpp)
target_link_libraries(${PROJECT_NAME}_hann ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})

# Hamming Window
add_library(${PROJECT_NAME}_hamming src/Hamming.cpp)
target_link_libraries(${PROJECT_NAME}_hamming ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})

# Flattop Window
add_library(${PROJECT_NAME}_flattop src/Flattop.cpp)
target_link_libraries(${PROJECT_NAME}_flattop ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})

# Blackman Window
add_library(${PROJECT_NAME}_blackman src/Blackman.cpp)
target_link_libraries(${PROJECT_NAME}_blackman ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})

#################
## Executables ##
#################

add_executable(test_car test/test_car.cpp)
target_link_libraries(test_car ${PROJECT_NAME}_car ${catkin_LIBRARIES})

add_executable(test_dc test/test_dc.cpp)
target_link_libraries(test_dc ${PROJECT_NAME}_dc ${catkin_LIBRARIES})

add_executable(test_hann test/test_hann.cpp)
target_link_libraries(test_hann ${PROJECT_NAME}_hann ${catkin_LIBRARIES})

add_executable(test_hamming test/test_hamming.cpp)
target_link_libraries(test_hamming ${PROJECT_NAME}_hamming ${catkin_LIBRARIES})

add_executable(test_flattop test/test_flattop.cpp)
target_link_libraries(test_flattop ${PROJECT_NAME}_flattop ${catkin_LIBRARIES})

add_executable(test_blackman test/test_blackman.cpp)
target_link_libraries(test_blackman ${PROJECT_NAME}_blackman ${catkin_LIBRARIES})

add_executable(test_chain test/test_chain.cpp)
target_link_libraries(test_chain ${PROJECT_NAME}_car ${PROJECT_NAME}_dc ${catkin_LIBRARIES}) 

add_executable(test_laplacian test/test_laplacian.cpp)
target_link_libraries(test_laplacian ${PROJECT_NAME}_laplacian ${catkin_LIBRARIES}) 

#add_executable(test test/test.cpp)
#target_link_libraries(test ${PROJECT_NAME}_car ${PROJECT_NAME}_dc ${catkin_LIBRARIES}) 
