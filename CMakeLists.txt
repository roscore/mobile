cmake_minimum_required(VERSION 2.8.3)
project(mobile_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ssoni_mobile_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES mobile_robot
	CATKIN_DEPENDS roscpp std_msgs ssoni_mobile_msgs
	)

include_directories(include ${catkin_INCLUDE_DIRS} )

 add_executable(motor_node src/main.cpp src/motor_algorithm.cpp)
 target_link_libraries(motor_node ${catkin_LIBRARIES} pigpiod_if2)
 add_dependencies(motor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
