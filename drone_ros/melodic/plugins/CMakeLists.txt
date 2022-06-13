cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find all the required components
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(${catkin_INCLUDE_DIRS})

# Build our plugin
add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

#add_library(test SHARED test.cc)
#target_link_libraries(test ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
