cmake_minimum_required(VERSION 2.8.3)

project(robocontroller)

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  message_generation
  geometry_msgs
)

find_package( PkgConfig REQUIRED )
pkg_check_modules ( ncurses++ REQUIRED ncurses++ )

#########################################################
# MESSAGES
add_message_files(
	FILES    
	Pose.msg
	Debug.msg
	Telemetry.msg
	Debug_00_23.msg
	Debug_50_53.msg
	Debug_200_220.msg
	Debug_250_259.msg
	Debug_60000_60019.msg
)

# SERVICES
add_service_files(
   FILES
   SetPID.srv
   GetPID.srv
   EnablePID.srv
   EnableCommWD.srv
   GetBoardStatus.srv
   SetBatteryCalib.srv
   EnableSaveToEeprom.srv
   SetRegister.srv
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
    src/rcteleop/include/
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

set(robocontroller_teleop_SRC
    src/robocontroller_teleop_key_node.cpp
    src/rcteleop/src/rc_teleop_key.cpp
)

set(robocontroller_debug_SRC
	src/robocontroller_debug_node.cpp
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

add_dependencies(
	robocontroller_node 
	${PROJECT_NAME}_generate_messages_cpp
)

add_executable( 
        robocontroller_teleop_key_node
	${robocontroller_teleop_SRC}
)

add_dependencies(
	robocontroller_teleop_key_node 
	${PROJECT_NAME}_generate_messages_cpp
)

add_executable( 
        robocontroller_debug_node
	${robocontroller_debug_SRC}
)

add_dependencies(
	robocontroller_debug_node 
	${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(
	robocontroller_node 
	${catkin_LIBRARIES} 
	modbus
)

target_link_libraries(
        robocontroller_teleop_key_node
	${catkin_LIBRARIES}	
	ncurses
)

target_link_libraries(
	robocontroller_debug_node 
	${catkin_LIBRARIES} 
	modbus
)






#########################################################
