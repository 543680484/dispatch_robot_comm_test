cmake_minimum_required(VERSION 2.8.3)
project(dispatch_robot_comm)

add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    actionlib_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
include_directories("${JSONCPP_INCLUDE_DIRS}")
#link_libraries(${JSONCPP_LIBRARIES})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros2json
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dispatch_robot_lib src/dispatch_robot_comm.cpp)
target_link_libraries(dispatch_robot_lib
    ${catkin_LIBRARIES}
    ${JSONCPP_LIBRARIES})

add_executable(dispatch_robot_comm src/main.cpp)
target_link_libraries(dispatch_robot_comm
    dispatch_robot_lib
    )

add_executable(local_sdk_server test/local_sdk_server.cpp)
target_link_libraries(local_sdk_server
    dispatch_robot_lib
    )
