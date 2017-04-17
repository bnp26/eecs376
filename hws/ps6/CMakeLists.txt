cmake_minimum_required(VERSION 2.8.3)
project(eecs376_ps6)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(${PROJECT_NAME}_ps6_path_client src/pub_des_state_ps6_path_client.cpp)

cs_install()
cs_export()


