cmake_minimum_required(VERSION 2.8.3)
project(eecs376_ps3)

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

#################
## SERVICE LIB ##
#################
set( ${PROJECT_NAME}_PATH_SERVICES_SOURCES
	src/PathExecutor.cc
	src/movement_commands.cc
)

set( ${PROJECT_NAME}_PATH_SERVICES_PUBLIC_HEADERS
)

set( ${PROJECT_NAME}_PATH_SERVICES_PRIVATE_HEADERS
	include/${PROJECT_NAME}/PathExecutor.h
	include/${PROJECT_NAME}/movement_commands.h
)

## Declare a C++ library
cs_add_library(${PROJECT_NAME}_path_services
  	${${PROJECT_NAME}_PATH_SERVICES_SOURCES} ${${PROJECT_NAME}_PATH_SERVICES_PRIVATE_HEADERS} ${${PROJECT_NAME}_PATH_SERVICES_PUBLIC_HEADERS}
)


##################
## SERVICE NODE ##
##################
cs_add_executable( ${PROJECT_NAME}_path_services_node src/path_service.cc )
target_link_libraries( ${PROJECT_NAME}_path_services_node
	${PROJECT_NAME}_path_services
)

#################
## CLIENT NODE ##
#################
set( ${PROJECT_NAME}_PATH_CLIENT_NODE_SRCS
	src/waypoint_generator.cc
	src/path_client.cc
)

set( ${PROJECT_NAME}_PATH_CLIENT_NODE_HEADERS
	include/${PROJECT_NAME}/waypoint_generator.h
)

cs_add_executable( ${PROJECT_NAME}_path_client_node
	${${PROJECT_NAME}_PATH_CLIENT_NODE_SRCS} ${${PROJECT_NAME}_PATH_CLIENT_NODE_HEADERS}
)

cs_install()
cs_export()
