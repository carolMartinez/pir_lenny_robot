cmake_minimum_required(VERSION 2.8.3)
project(lenny_motion)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED
	roscpp
	lenny_msgs
	moveit_core
	#geometric_shapes
	moveit_ros_planning_interface
	moveit_msgs
	#shape_msgs
	std_msgs
	#apc16delft_grasp_synthesizer
	#dr_eigen
	#dr_param
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS
	roscpp
	moveit_ros_planning_interface
	moveit_msgs
	lenny_msgs
	#dr_eigen
	#dr_param
)


include_directories(
 include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
	src/motion_executor.cpp
)


add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
	yaml-cpp
)

add_executable(motion_executor src/motion_executor.cpp)

target_link_libraries(motion_executor
	${PROJECT_NAME}
	${catkin_LIBRARIES}
)

