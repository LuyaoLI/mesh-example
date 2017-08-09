cmake_minimum_required(VERSION 2.8.3)
project(map_visualization_2)

find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
  roscpp
	rospy
	roslaunch
	pcl_ros
	visualization_msgs
	interactive_markers
	geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS
	sensor_msgs
	roscpp
	rospy
	pcl_ros
	visualization_msgs
	interactive_markers
	geometry_msgs
)

find_package(octomap REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.2 REQUIRED)

roslaunch_add_file_check(launch)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OCTOMAP_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(load_map src/load_map.cpp)
target_link_libraries(load_map ${catkin_LIBRARIES})
