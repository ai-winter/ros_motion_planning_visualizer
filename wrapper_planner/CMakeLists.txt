cmake_minimum_required(VERSION 3.0.2)
project(wrapper_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  tf2_geometry_msgs
  tf2_ros
  message_generation
  evolutionary_planner
  global_planner
  graph_planner
  sample_planner
  voronoi_layer
)

add_service_files(
  FILES
  CallPlan.srv
  )
  
generate_messages(
  DEPENDENCIES
  geometry_msgs
  )
    
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS global_planner graph_planner sample_planner voronoi_layer evolutionary_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/wrapper_planner.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
