cmake_minimum_required(VERSION 2.8.3)
project(ros_omron_agv)

find_package(catkin REQUIRED COMPONENTS
  angles
  actionlib
  actionlib_msgs
  libaria
  move_base_msgs
  nav_msgs
	roscpp
  sensor_msgs
  std_msgs
	std_srvs
	tf
	tf2
	tf2_eigen
	tf2_ros
	message_generation
)

find_package(Boost REQUIRED COMPONENTS thread)

add_message_files(
  FILES
  Omron.msg
)

add_service_files(
  FILES
  DockRequest.srv
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
  ${catkin_INCLUDE_DIRS}
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

#############
## Install ##
#############

install(TARGETS omron_laser_node robot_status_node map_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ rviz/ docs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)