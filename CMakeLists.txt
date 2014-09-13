cmake_minimum_required(VERSION 2.8.3)

project(robocontroller)

set(ROS_BUILD_TYPE Debug)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp)

# Declare our catkin package.
catkin_package()



set(robocontroller_SRC
    src/robocontroller_node.cpp
    src/rbctrliface/src/rbctrliface.cpp
    src/rbctrliface/src/robotctrl.cpp
)

set(robocontroller_INC_DIRS
    src/
    src/rbctrliface/include/
)

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS} ${robocontroller_INC_DIRS})

add_executable(robocontroller_node ${robocontroller_SRC})

target_link_libraries(robocontroller_node ${catkin_LIBRARIES} modbus)

