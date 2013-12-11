cmake_minimum_required(VERSION 2.8.3)
project(sr_point_cloud)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  message_generation
  rostest
  dynamic_reconfigure
  actionlib_msgs
  genmsg
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system)
find_package(orocos_kdl REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   #Foo.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES Track.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
)

###################################
##      dynamic reconfigure      ##
###################################
generate_dynamic_reconfigure_options(
  cfg/Tracker.cfg
  cfg/Triangulator.cfg
  cfg/ClusterSegmentor.cfg
  #...
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES
  CATKIN_DEPENDS message_runtime
  DEPENDS boost orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# TODO: Check names of system library include directories (boost)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(sr_point_cloud
#   src/${PROJECT_NAME}/sr_point_cloud.cpp
# )

# bin/tracker #
######################

## Declare a cpp executable
add_executable(tracker
  src/tracker.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(tracker
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_gencpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(tracker
  ${catkin_LIBRARIES}
  ${boost_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
)

# Work around a bug in kdl cmake. Without this we get error:
# /usr/bin/ld: cannot find -lorocos-kdl
# See: http://bugs.orocos.org/show_bug.cgi?id=1035
set_target_properties(tracker
  PROPERTIES LINK_FLAGS
  -L${orocos_kdl_LIBRARY_DIRS}
)

# triangulator #
#######################

add_executable(triangulator
  src/triangulator.cpp
)

add_dependencies(triangulator
  ${PROJECT_NAME}_gencfg
  sr_grasp_msgs_gencpp
)

target_link_libraries(triangulator
  ${catkin_LIBRARIES}
  ${boost_LIBRARIES}
)

# cluster_segmentor #
#####################

add_executable(cluster_segmentor src/cluster_segmentor_node.cpp)

add_dependencies(cluster_segmentor ${PROJECT_NAME}_gencfg)

target_link_libraries(cluster_segmentor
  ${catkin_LIBRARIES}
  ${boost_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tracker tracker_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
add_rostest_gtest(test_triangulator
  test/test_triangulator.test
  test/test_triangulator.cpp
)
target_link_libraries(test_triangulator
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)
add_dependencies(test_triangulator
  sr_grasp_msgs_gencpp
  ${catkin_EXPORTED_TARGETS}
)

add_rostest(test/cloud_tracker.test)
add_rostest(test/cluster_segmentor.test)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
