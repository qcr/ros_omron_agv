cmake_minimum_required(VERSION 2.8.3)
project(omron_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
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

add_executable(driver_node src/pub_laser.cpp)

target_link_libraries(driver_node
  ${catkin_LIBRARIES} Aria ArNetworking
)

install(TARGETS driver_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})