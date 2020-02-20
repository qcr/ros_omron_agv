cmake_minimum_required(VERSION 2.8.3)
project(rv_omron_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
	tf
)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Aria QUIET)
if(Aria_FOUND)
	message(STATUS "Got Aria")
	if(EXISTS "${Aria_INCLUDE_DIRS}/Aria.h")
		add_definitions(-DADEPT_PKG)
	endif()
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
else()
	message(STATUS "Didn't get Aria")
	# The installation package provided by Adept doesn't follow Debian policies
	if(EXISTS "/usr/local/Aria/include/Aria.h")
	  message(STATUS "Manually added aria")
	  add_definitions(-DADEPT_PKG)
	  include_directories( /usr/local/Aria/include)
	  link_directories( /usr/local/Aria/lib )
	endif()
endif()

find_package(ArNetworking QUIET)
if(ArNetworking_FOUND)
	if(EXISTS "${ArNetworking_INCLUDE_DIRS}/ArNetworking.h")
		add_definitions(-DAR_PKG)
	endif()
  include_directories(${ArNetworking_INCLUDE_DIRS})
  link_directories(${ArNetworking_LIBRARY_DIRS})
else()
	# The installation package provided by Adept doesn't follow Debian policies
	if(EXISTS "/usr/local/Aria/ArNetworking/include/ArNetworking.h")
	  message(STATUS "Manually added ArNet")
	  add_definitions(-DAR_PKG)
	  include_directories( /usr/local/Aria/ArNetworking/include)
	  link_directories(/usr/local/Aria/ArNetworking/lib )
	endif()
endif()


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
  ${catkin_LIBRARIES} Aria ArNetworking
)

target_link_libraries(robot_status_node
  ${catkin_LIBRARIES} Aria ArNetworking
)

target_link_libraries(map_node
  ${catkin_LIBRARIES} Aria ArNetworking
)
