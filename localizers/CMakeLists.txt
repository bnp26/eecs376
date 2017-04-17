cmake_minimum_required(VERSION 2.8.3)
project(localizers)

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

set(${PROJECT_NAME}_w_amcl_headers
    include/${PROJECT_NAME}/odom_tf.h
)

set(${PROJECT_NAME}_w_amcl_srcs
    src/odom_tf_fncs_w_amcl.cc
    src/localizer_node.cc
)

cs_add_executable(${PROJECT_NAME}_w_amcl
    ${${PROJECT_NAME}_w_amcl_headers}
    ${${PROJECT_NAME}_w_amcl_srcs}
)

set(${PROJECT_NAME}_no_amcl_headers
    include/${PROJECT_NAME}/odom_tf.h
)

set(${PROJECT_NAME}_no_amcl_srcs
    src/odom_tf_fncs_no_amcl.cc
    src/localizer_node.cc
)

cs_add_executable(${PROJECT_NAME}_no_amcl
    ${${PROJECT_NAME}_no_amcl_headers}
    ${${PROJECT_NAME}_no_amcl_srcs}
)

#target_link_libraries(${PROJECT_NAME}_linear_steering_controller
#    ${catkin_LIBRARIES}
#)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)

cs_install()
cs_export()

