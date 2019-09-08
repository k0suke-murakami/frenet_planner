cmake_minimum_required(VERSION 2.8.3)
project(frenet_planner)

add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/distance_transform/extern/DopeVector ${CMAKE_CURRENT_BINARY_DIR}/lib_dope_vector)
include(${CMAKE_CURRENT_SOURCE_DIR}/lib/distance_transform/extern/DopeVector/sources_properties.cmake)


set(toLink DopeVector)
if(CMAKE_COMPILER_IS_GNUCXX)
find_package(Threads REQUIRED)
list(APPEND toLink Threads::Threads)
endif()



set(hdr_dir ${CMAKE_CURRENT_SOURCE_DIR}/lib/distance_transform/include)

set(hdr_inline_files
${hdr_dir}/distance_transform/inlines/distance_transform.inl
)
set_source_files_properties(${hdr_inline_files} PROPERTIES XCODE_EXPLICIT_FILE_TYPE "sourcecode.cpp.h")
source_group("distance_transform\\inlines" FILES ${hdr_inline_files})

set(hdr_main_files
${hdr_dir}/distance_transform/distance_transform.hpp
)
source_group("distance_transform" FILES ${hdr_main_files})

set(all_hdr ${hdr_inline_files} ${hdr_main_files})



add_library(distance_transform INTERFACE)

target_include_directories(distance_transform INTERFACE ${hdr_dir})
target_link_libraries(distance_transform INTERFACE ${toLink})

if(ATTACH_SOURCES)
target_sources(distance_transform INTERFACE ${all_hdr})
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  autoware_msgs
  vector_map
  grid_map_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  tf2_ros
  autoware_msgs
  vector_map
  grid_map_ros
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
  src/modified_reference_path_generator.cpp
)

target_link_libraries(frenet_planner
  ${catkin_LIBRARIES}
  distance_transform
)

add_dependencies(frenet_planner
  ${catkin_EXPORTED_TARGETS}
  distance_transform
)

install(TARGETS
        frenet_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
