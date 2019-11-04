cmake_minimum_required(VERSION 2.8.3)
project(imuViz)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)



find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	pcl_conversions 
	pcl_ros
	sensor_msgs
)



catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(main src/viz.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ${PCL_LIBRARIES}${EIGEN3_LIBRARIES})


