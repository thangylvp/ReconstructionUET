cmake_minimum_required(VERSION 2.8.3)
project(PclVisualization)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.8 REQUIRED)


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


add_executable(viz src/viz.cpp)
target_link_libraries(viz ${catkin_LIBRARIES} ${PCL_LIBRARIES})


