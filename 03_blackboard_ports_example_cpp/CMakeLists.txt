cmake_minimum_required(VERSION 3.5)

project(blackboard_ports)

# set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp REQUIRED)

add_executable(blackboard_bt "BlackBoardPortsExample.cpp")
target_link_libraries(blackboard_bt BT::behaviortree_cpp)