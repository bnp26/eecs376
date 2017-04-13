cmake_minimum_required(VERSION 2.8.3)
project(eecs376_ps7)

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

cs_add_executable(${PROJECT_NAME}_ps7_path_client src/pub_des_state_ps7_path_client.cc)
# target_link_libraries(${PROJECT_NAME}_open_loop_controller ${catkin_LIBRARIES})

#target_link_libraries(${PROJECT_NAME}_linear_steering_controller
#    ${catkin_LIBRARIES}
#)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)

cs_install()
cs_export()

