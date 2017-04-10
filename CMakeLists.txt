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
  message_generation
  geometry_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(Threads)
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread -lpthread -Wl,--no-as-needed")

add_message_files(
  DIRECTORY msg
  FILES
  pressure_read.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(

  CATKIN_DEPENDS message_runtime

)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pad_scan 
  src/main.cpp
  src/decompresser.cpp
  src/force_generator.cpp
  src/slip_detect.cpp

  include/pressure_pad/decompresser.h
  include/pressure_pad/force_generator.h
  include/pressure_pad/slip_detect.h

)

target_link_libraries(pad_scan ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
add_dependencies(pad_scan ${catkin_EXPORTED_TARGETS})
add_definitions(-std=c++11)
