cmake_minimum_required(VERSION 2.8.3)
project(planners)

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

set( ${PROJECT_NAME}_TRAJ_SERVICE_SRCS
    src/utils.cc
    src/trajectory_builder.cc
    src/pub_des_state.cc
    src/traj_service_main.cc
)

set( ${PROJECT_NAME}_TRAJ_SERVICE_HEADERS
    include/${PROJECT_NAME}/utils.h
    include/${PROJECT_NAME}/types.h
    include/${PROJECT_NAME}/trajectory_builder.h
    include/${PROJECT_NAME}/pub_des_state.h
)

cs_add_executable( ${PROJECT_NAME}_traj_service
    ${${PROJECT_NAME}_TRAJ_SERVICE_SRCS} ${${PROJECT_NAME}_TRAJ_SERVICE_HEADERS}
)

cs_install()
cs_export()
