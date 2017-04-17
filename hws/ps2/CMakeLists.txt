cmake_minimum_required(VERSION 2.8.3)
project(eecs376_ps2)

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

set( ${PROJECT_NAME}_REACTIVE_COMMANDER_SRCS
	src/movement_commands.cc
	src/reactive_commander.cc
)

set( ${PROJECT_NAME}_REACTIVE_COMMANDER_HEADERS
	include/${PROJECT_NAME}/movement_commands.h
)

cs_add_executable( ${PROJECT_NAME}_reactive_commander_node
	${${PROJECT_NAME}_REACTIVE_COMMANDER_SRCS} ${${PROJECT_NAME}_REACTIVE_COMMANDER_HEADERS}
)

cs_install()
cs_export()
