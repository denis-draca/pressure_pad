cmake_minimum_required(VERSION 2.8.3)
project(pressure_pad)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
find_package(Threads)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread -lpthread -Wl,--no-as-needed")

catkin_package(
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pad_scan 
  src/main.cpp
  src/decompresser.cpp

  include/pressure_pad/decompresser.h

)

target_link_libraries(pad_scan ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
add_definitions(-std=c++11)