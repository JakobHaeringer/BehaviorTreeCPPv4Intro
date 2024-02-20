cmake_minimum_required(VERSION 3.0.2) # 
project(bt_bfmc_23)
add_compile_options(-std=c++17 -Wall -O2) #

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    behaviortree_cpp
)

# Make sure to always put this before adding your executable! Otherwise your package will not be shown in /devel/lib/<package_name>
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp behaviortree_cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_libraries(${catkin_LIBRARIES})

set(target "planner")

add_executable("${target}" 
  src/planner_node.cpp
)
target_include_directories("${target}" PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries("${target}" ${catkin_LIBRARIES})

