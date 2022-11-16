cmake_minimum_required(VERSION 3.0.2)
project(air_arm_hardware_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  air_arm_robot
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS
   include

 LIBRARIES
   air_arm_hardware_interface
   air_arm_robot

 CATKIN_DEPENDS
   controller_manager
   hardware_interface
   air_arm_robot
   roscpp
  DEPENDS AdsLib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../ADS/AdsLib
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/air_arm_hw.cpp )
#add_dependencies(${PROJECT_NAME} air_arm_robot)
target_link_libraries(${PROJECT_NAME}
    ${air_arm_robot_LIBRARIES}
    ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/air_arm_hardware_interface_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES})


