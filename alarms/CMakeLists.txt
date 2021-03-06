cmake_minimum_required(VERSION 2.8.3)
project(alarms)

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

set(${PROJECT_NAME}_laser_alarm_lib_srcs
    src/core/laser_parser.cc
    src/laser_alarm_node.cc
)

set(${PROJECT_NAME}_laser_alarm_lib_headers
    include/${PROJECT_NAME}/core/laser_parser.h
)

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(${PROJECT_NAME}_laser_alarm_lib
    ${${PROJECT_NAME}_laser_alarm_lib_srcs} ${${PROJECT_NAME}_laser_alarm_lib_headers}
)


# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(${PROJECT_NAME}_laser_alarm_node src/laser_alarm_node.cc)
cs_add_executable(${PROJECT_NAME}_estop_alarm_node src/estop_alarm_node.cc)


#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(${PROJECT_NAME}_laser_alarm_node ${PROJECT_NAME}_laser_alarm_lib)

cs_install()
cs_export()

#############
## Testing ##
#############

set( ${PROJECT_NAME}_UNIT_SRCS
	test/laser_parser_unit.cc
	test/main.cc
)

set( ${PROJECT_NAME}_UNIT_HEADERS

)

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test ${${PROJECT_NAME}_UNIT_SRCS} ${${PROJECT_NAME}_UNIT_HEADERS})
if(TARGET ${PROJECT_NAME}-test)
	target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_lidar_alarm)
endif()
