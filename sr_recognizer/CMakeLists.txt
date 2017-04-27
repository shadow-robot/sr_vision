cmake_minimum_required(VERSION 2.8.3)
project(sr_recognizer)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy
  actionlib 
  actionlib_msgs std_msgs geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package( PCL 1.7 REQUIRED )
find_package( OpenCV 2.4 REQUIRED )
find_package( V4R REQUIRED COMPONENTS v4r_apps)

add_action_files(
  DIRECTORY action
  FILES Recognizer.action
)

generate_messages(
  DEPENDENCIES 
  actionlib_msgs 
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  actionlib_msgs
  std_msgs
  geometry_msgs
)

add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include 
 ${catkin_INCLUDE_DIRS} 
 ${Boost_INCLUDE_DIRS} 
 ${PCL_INCLUDE_DIRS}
 ${V4R_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(recognizer_server src/recognizer_server.cpp)

target_link_libraries(
  recognizer_server
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${V4R_LIBS}
)

add_dependencies(
  recognizer_server
  ${sr_recognizer_EXPORTED_TARGETS}
)

