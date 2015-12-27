cmake_minimum_required(VERSION 2.8.3)
project(inverted_pendulum)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(inverted_pendulum_publisher src/inverted_pendulum_publisher.cpp)
cs_add_executable(inverted_pendulum_disturbance src/inverted_pendulum_disturbance.cpp)
cs_add_executable(inverted_pendulum_controller src/inverted_pendulum_controller.cpp)
# target_link_libraries(example ${catkin_LIBRARIES})

cs_install()
cs_export()
