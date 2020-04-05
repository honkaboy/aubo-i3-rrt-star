cmake_minimum_required(VERSION 3.5)

project (pathplanner_service)


#You may add sources to build here
add_library(path_planner 
src/rrt_star_planner.cpp
src/tree.cpp
src/node.cpp
src/utilities.cpp
src/aubo_i3_kinematics.cpp
src/robot_api.cpp)

#you may add additional include directories here
target_include_directories(path_planner PUBLIC inc/ /usr/local/include/eigen3)

#You may link in additional libraries if necessary

#builds application to call planner function
add_executable(path_planner_test src/planner_main.cpp) 
target_include_directories(path_planner_test PUBLIC inc/ /usr/local/include/eigen3)
add_dependencies(path_planner_test path_planner)
target_link_libraries(path_planner_test path_planner)
