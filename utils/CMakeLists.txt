cmake_minimum_required(VERSION 2.8.3)
project(utils)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next 2 lines to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})


#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

set(math_utils_lib_src

)

set(math_utils_lib_headers

)
# cs_add_library(math_utils
#     ${math_utils_lib_srcs} ${math_utils_lib_headers}
# )



set(message_utils_lib_srcs

)

set(message_utils_lib_headers

)
# cs_add_library(message_utils
#     ${message_utils_lib_srcs} ${message_utils_lib_headers}
# )
# target_link_libraries(message_utils math_utils)


set(transform_utils_lib_srcs

)

set(transform_utils_lib_headers

)
# cs_add_library(transform_utils
#     ${transform_utils_lib_srcs} ${transform_utils_lib_srcs}
# )
# target_link_libraries(transform_utils math_utils)

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(${PROJECT_NAME}_laser_alarm_lib
#     ${${PROJECT_NAME}_laser_alarm_lib_srcs} ${${PROJECT_NAME}_laser_alarm_lib_headers}
# )


# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
# cs_add_executable(${PROJECT_NAME}_laser_alarm_node src/laser_alarm_node.cc)


#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(${PROJECT_NAME}_laser_alarm_node ${PROJECT_NAME}_laser_alarm_lib)

cs_install()
cs_export()

