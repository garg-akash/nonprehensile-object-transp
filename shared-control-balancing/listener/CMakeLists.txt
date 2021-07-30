#cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#find_package(gazebo REQUIRED)
#find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
#include_directories(${GAZEBO_INCLUDE_DIRS})
#include_directories(${catkin_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_library(listener SHARED listener.cc)
#target_link_libraries(listener ${GAZEBO_LIBRARIES})

#add_library(stepControl SHARED stepControl.cc)
#target_link_libraries(stepControl ${GAZEBO_LIBRARIES})
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(listener listener.cc)
target_link_libraries(listener ${GAZEBO_LIBRARIES} pthread)

add_executable(stepControl stepControl.cc)
target_link_libraries(stepControl ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} pthread)
