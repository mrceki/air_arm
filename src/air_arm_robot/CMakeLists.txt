cmake_minimum_required(VERSION 3.0.2)
project(air_arm_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES air_arm_robot
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ../ADS/AdsLib
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/arm.cpp
  src/arm_ik.cpp
  ../ADS/AdsLib/standalone/AdsLib.cpp
  ../ADS/AdsLib/standalone/AmsConnection.cpp
  ../ADS/AdsLib/standalone/AmsNetId.cpp
  ../ADS/AdsLib/standalone/AmsPort.cpp
  ../ADS/AdsLib/standalone/AmsRouter.cpp
  ../ADS/AdsLib/standalone/NotificationDispatcher.cpp
  ../ADS/AdsLib/AdsDef.cpp
  ../ADS/AdsLib/AdsDevice.cpp
  ../ADS/AdsLib/AdsFile.cpp
  ../ADS/AdsLib/Log.cpp
  ../ADS/AdsLib/Sockets.cpp
  ../ADS/AdsLib/Frame.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

