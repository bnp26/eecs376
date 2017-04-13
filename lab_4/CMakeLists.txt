cmake_minimum_required(VERSION 2.8.3)
project(lab_4)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  genmsg
  actionlib_msgs
  actionlib
  # message_generation
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES
  Path.action
#  Action2.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES eecs376_ps4
#  CATKIN_DEPENDS geometry_msgs message_generation roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

#################
## LIDAR ALARM ##
#################

set( ${PROJECT_NAME}_LIDAR_ALARM_LIB_SRCS
	src/lidar_parser.cc
)

set( ${PROJECT_NAME}_LIDAR_ALARM_LIB_HEADERS
	include/${PROJECT_NAME}/lidar_parser.h
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lidar_alarm_lib
  ${${PROJECT_NAME}_LIDAR_ALARM_LIB_SRCS} ${${PROJECT_NAME}_LIDAR_ALARM_LIB_HEADERS}
)
target_link_libraries( ${PROJECT_NAME}_lidar_alarm_lib
	${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(eecs376_ps4 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(${PROJECT_NAME}_lidar_alarm_node src/lidar_alarm.cc)
target_link_libraries( ${PROJECT_NAME}_lidar_alarm_node
	${PROJECT_NAME}_lidar_alarm_lib
	${catkin_LIBRARIES}
)

###################
## ACTION SERVER ##
###################

set( ${PROJECT_NAME}_ACTION_SERVER_LIB_SRCS
	src/PathExecutor.cc
	src/ActionServer.cc
)

set( ${PROJECT_NAME}_ACTION_SERVER_LIB_HEADERS
	include/${PROJECT_NAME}/PathExecutor.h
	include/${PROJECT_NAME}/ActionServer.h
)

add_library( ${PROJECT_NAME}_action_server_lib
	${${PROJECT_NAME}_ACTION_SERVER_LIB_SRCS} ${${PROJECT_NAME}_ACTION_SERVER_LIB_HEADERS}
)
target_link_libraries( ${PROJECT_NAME}_action_server_lib
	${catkin_LIBRARIES}
)

add_executable( ${PROJECT_NAME}_action_server_node
	src/action_server_node.cc
)
target_link_libraries( ${PROJECT_NAME}_action_server_node
	${PROJECT_NAME}_action_server_lib
	${catkin_LIBRARIES}
)

###################
## ACTION CLIENT ##
###################

set( ${PROJECT_NAME}_ACTION_CLIENT_NODE_SRCS
	src/waypoint_generator.cc
	src/action_client_node.cc
)

set( ${PROJECT_NAME}_ACTION_CLIENT_NODE_HEADER
	include/${PROJECT_NAME}/waypoint_generator.h
)

add_executable( ${PROJECT_NAME}_action_client_node
	${${PROJECT_NAME}_ACTION_CLIENT_NODE_SRCS} ${${PROJECT_NAME}_ACTION_CLIENT_NODE_HEADER}
)
target_link_libraries( ${PROJECT_NAME}_action_client_node
	${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(eecs376_ps4_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(eecs376_ps4_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS eecs376_ps4 eecs376_ps4_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_eecs376_ps4.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
