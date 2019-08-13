cmake_minimum_required(VERSION 2.8.3)
project(frenet_planner)

add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  grid_map_ros
  autoware_msgs
  vector_map
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  tf2_ros
  grid_map_ros
  autoware_msgs
  vector_map
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

)

add_executable(frenet_planner
  src/frenet_planner_node.cpp
  src/frenet_planner_ros.cpp
  src/frenet_planner.cpp
  src/vectormap_ros.cpp
  src/calculate_center_line.cpp
)

target_link_libraries(frenet_planner
  ${catkin_LIBRARIES}
)

add_dependencies(frenet_planner
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
        frenet_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        
# install(DIRECTORY launch/
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#         PATTERN ".svn" EXCLUDE)
