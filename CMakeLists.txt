cmake_minimum_required(VERSION 2.8.3)
project(rv_omron_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
	std_srvs
	tf
	tf2
	tf2_ros
	tf2_eigen
	rv_msgs
	rv_tasks
	rv_trees
	actionlib
	actionlib_msgs
	move_base_msgs
	message_generation
	rv_libaria
)

find_package(Boost REQUIRED COMPONENTS thread)

add_message_files(
  FILES
  Omron.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package()

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  src
)


add_executable(omron_laser_node src/laser_node.cpp)
add_executable(robot_status_node src/robot_status.cpp)
add_executable(map_node src/map_server.cpp)

target_link_libraries(omron_laser_node
  ${catkin_LIBRARIES}
)

target_link_libraries(robot_status_node
  ${catkin_LIBRARIES}
)

target_link_libraries(map_node
  ${catkin_LIBRARIES}
)
