cmake_minimum_required(VERSION 2.8.3)
project(obstacle_extractor)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(obstacle_extractor_lib
    src/obstacle_extractor.cpp
)
target_link_libraries(obstacle_extractor_lib ${catkin_LIBRARIES})

cs_add_executable(obstacle_extractor_node src/obstacle_extractor_node.cpp)
target_link_libraries(obstacle_extractor_node obstacle_extractor_lib)

cs_install()
cs_export()
