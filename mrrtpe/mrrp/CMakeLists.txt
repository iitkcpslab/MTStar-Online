cmake_minimum_required(VERSION 2.8.3)
project(mrrp)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(mrrp_lib
    src/mrrp.cpp
)
target_link_libraries(mrrp_lib ${catkin_LIBRARIES})

cs_add_executable(mrrp_node src/mrrp_node.cpp)
target_link_libraries(mrrp_node mrrp_lib)

cs_install()
cs_export()
