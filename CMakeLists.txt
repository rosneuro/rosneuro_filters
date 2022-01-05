cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_filters)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(ROSNEURO_DATA_MIN_VERSION "1.0.0")

##############################################################################
# Find dependencies
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
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
  LIBRARIES ${PROJECT_NAME}_car ${PROJECT_NAME}_dc
  CATKIN_DEPENDS
  	roscpp
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
add_library(${PROJECT_NAME}_car include/${PROJECT_NAME}/Car.hpp)
set_target_properties(${PROJECT_NAME}_car PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME}_car ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_car ${catkin_EXPORTED_TARGETS})

# DC Spatial filter
add_library(${PROJECT_NAME}_dc include/${PROJECT_NAME}/Dc.hpp)
set_target_properties(${PROJECT_NAME}_dc PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME}_dc ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_dc ${catkin_EXPORTED_TARGETS})

# Hann Window
add_library(${PROJECT_NAME}_hann include/${PROJECT_NAME}/Hann.hpp)
set_target_properties(${PROJECT_NAME}_hann PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME}_hann ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_hann ${catkin_EXPORTED_TARGETS})

# Hamming Window
add_library(${PROJECT_NAME}_hamming include/${PROJECT_NAME}/Hamming.hpp)
set_target_properties(${PROJECT_NAME}_hamming PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME}_hamming ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_hamming ${catkin_EXPORTED_TARGETS})

# Flattop Window
add_library(${PROJECT_NAME}_flattop include/${PROJECT_NAME}/Flattop.hpp)
set_target_properties(${PROJECT_NAME}_flattop PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME}_flattop ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_flattop ${catkin_EXPORTED_TARGETS})

# Blackman Window
add_library(${PROJECT_NAME}_blackman include/${PROJECT_NAME}/Blackman.hpp)
set_target_properties(${PROJECT_NAME}_blackman PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME}_blackman ${Eigen3_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_blackman ${catkin_EXPORTED_TARGETS})

#################
## Executables ##
#################

add_executable(test_car test/test_car.cpp)
target_link_libraries(test_car ${PROJECT_NAME}_car)

add_executable(test_dc test/test_dc.cpp)
target_link_libraries(test_dc ${PROJECT_NAME}_dc)

add_executable(test_hann test/test_hann.cpp)
target_link_libraries(test_hann ${PROJECT_NAME}_hann)

add_executable(test_hamming test/test_hamming.cpp)
target_link_libraries(test_hamming ${PROJECT_NAME}_hamming)

add_executable(test_flattop test/test_flattop.cpp)
target_link_libraries(test_flattop ${PROJECT_NAME}_flattop)

add_executable(test_blackman test/test_blackman.cpp)
target_link_libraries(test_blackman ${PROJECT_NAME}_blackman)
