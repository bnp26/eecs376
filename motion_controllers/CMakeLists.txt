cmake_minimum_required(VERSION 2.8.3)
project(motion_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)

catkin_simple()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
# include_directories(
#    include
#    ${catkin_INCLUDE_DIRS}
#)

cs_add_executable(${PROJECT_NAME}_open_loop_controller src/open_loop_controller.cc)
# target_link_libraries(${PROJECT_NAME}_open_loop_controller ${catkin_LIBRARIES})

set(${PROJECT_NAME}_headers
    include/${PROJECT_NAME}/steering_algorithm.h
)

set(${PROJECT_NAME}_srcs
    src/lin_steering_wrt_odom.cc
)

cs_add_executable(${PROJECT_NAME}_linear_steering_controller
    ${${PROJECT_NAME}_headers}
    ${${PROJECT_NAME}_srcs}
)

#target_link_libraries(${PROJECT_NAME}_linear_steering_controller
#    ${catkin_LIBRARIES}
#)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)

cs_install()
cs_export()

