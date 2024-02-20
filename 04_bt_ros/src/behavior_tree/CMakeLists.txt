cmake_minimum_required(VERSION 3.0.2) # 
project(my_behaviortree)
add_compile_options(-std=c++17 -Wall -O2) #

set(target "bt_planner")

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    behaviortree_cpp
)

# Make sure to always put this before adding your executable! Otherwise your package will not be shown in /devel/lib/<package_name>
catkin_package(
  CATKIN_DEPENDS roscpp behaviortree_cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_libraries(${catkin_LIBRARIES})


add_executable("${target}" src/Finding_Ball.cpp)
target_include_directories("${target}" PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries("${target}" ${catkin_LIBRARIES})
