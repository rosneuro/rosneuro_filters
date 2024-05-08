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
find_package(GTest REQUIRED)
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
  ${PROJECT_NAME}
	
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

catkin_add_gtest(TestFilter test/TestFilter.cpp)
target_link_libraries(TestFilter ${GTEST_BOTH_LIBRARIES} pthread gmock ${Eigen3_LIBRARIES})
target_link_libraries(TestFilter ${PROJECT_NAME} ${catkin_LIBRARIES})
include_directories(${GTEST_INCLUDE_DIRS} gmock pthread)

catkin_add_gtest(TestFilterChain test/TestFilterChain.cpp)
target_link_libraries(TestFilterChain ${GTEST_BOTH_LIBRARIES} pthread gmock ${Eigen3_LIBRARIES})
target_link_libraries(TestFilterChain ${PROJECT_NAME} ${catkin_LIBRARIES})
include_directories(${GTEST_INCLUDE_DIRS} gmock pthread)

catkin_add_gtest(TestFilterChainNode test/TestFilterChainNode.cpp)
target_link_libraries(TestFilterChainNode ${GTEST_BOTH_LIBRARIES} pthread gmock ${Eigen3_LIBRARIES})
target_link_libraries(TestFilterChainNode ${PROJECT_NAME} ${catkin_LIBRARIES})
include_directories(${GTEST_INCLUDE_DIRS} gmock pthread)

include_directories(${catkin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
					include
					include/${PROJECT_NAME}/)

add_definitions(${EIGEN3_DEFINITIONS})

add_library(${PROJECT_NAME} src/Filter.cpp
							src/FilterChain.cpp
							src/FilterChainNode.cpp)

target_link_libraries(${PROJECT_NAME} ${Eigen3_LIBRARIES} 
									  ${catkin_LIBRARIES})

add_executable(filterchain_node example/RunFilterChainNode.cpp)
target_link_libraries(filterchain_node ${PROJECT_NAME} ${catkin_LIBRARIES}) 


#################
##   Install   ##
#################
install(TARGETS ${PROJECT_NAME}

	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS filterchain_node 
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

