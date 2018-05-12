cmake_minimum_required(VERSION 2.8.3)

project(tricycle_description)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	gazebo_plugins
	gazebo_ros
	urdf
     xacro
)
find_package(gazebo REQUIRED)
find_package(roslaunch)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs
	LIBRARIES
	TricycleDriveModPlugin
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

add_library(TricycleDriveModPlugin SHARED plugins/TricycleDriveModPlugin.cpp)
target_link_libraries(TricycleDriveModPlugin ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
