cmake_minimum_required(VERSION 3.5)

project(simple_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp REQUIRED)

add_executable(ball_bt "SimpleBallExample.cpp")
target_link_libraries(ball_bt BT::behaviortree_cpp)