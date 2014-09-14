cmake_minimum_required(VERSION 2.8.3)

project(robocontroller)

set(ROS_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  message_generation
  geometry_msgs
)

#########################################################
# MESSAGES
add_message_files(
    FILES
    Telemetry.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)
#########################################################

# Declare our catkin package.
catkin_package(
   INCLUDE_DIRS ${robocontroller_INC_DIRS}
#  LIBRARIES robocontroller
   CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs
#  DEPENDS system_lib
)

#########################################################
# INCLUDES
set(robocontroller_INC_DIRS
    src/
    src/rbctrliface/include/
)

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS} 
	${robocontroller_INC_DIRS}
)
#########################################################



#########################################################
# SOURCES
set(robocontroller_SRC
    src/robocontroller_node.cpp
    src/rbctrliface/src/rbctrliface.cpp
    src/rbctrliface/src/robotctrl.cpp
)
#########################################################


#########################################################
# EXECUTABLE
add_executable( 
	robocontroller_node 
	${robocontroller_SRC}
)

target_link_libraries(
	robocontroller_node 
	${catkin_LIBRARIES} 
	modbus
)
#########################################################

