cmake_minimum_required(VERSION 2.8.3)
project(eecs376_ps4)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

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

cs_add_library( ${PROJECT_NAME}_action_server_lib
	${${PROJECT_NAME}_ACTION_SERVER_LIB_SRCS} ${${PROJECT_NAME}_ACTION_SERVER_LIB_HEADERS}
)

cs_add_executable( ${PROJECT_NAME}_action_server_node
	src/action_server_node.cc
)
target_link_libraries( ${PROJECT_NAME}_action_server_node
	${PROJECT_NAME}_action_server_lib
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

cs_add_executable( ${PROJECT_NAME}_action_client_node
	${${PROJECT_NAME}_ACTION_CLIENT_NODE_SRCS} ${${PROJECT_NAME}_ACTION_CLIENT_NODE_HEADER}
)

cs_install()
cs_export()
