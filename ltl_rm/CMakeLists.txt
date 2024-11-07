cmake_minimum_required(VERSION 2.8.3)
project(ltl_rm)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# cs_add_library(rm_lib
#     src/rm.cpp
# )
cs_add_library(rm_lib
    src/manager_querygroup.cpp
)
target_link_libraries(rm_lib ${catkin_LIBRARIES})

# cs_add_executable(rm_node src/rm_node.cpp)
# target_link_libraries(rm_node rm_lib)
cs_add_executable(manager_node src/manager_querygroup_node.cpp)
cs_add_executable(quad src/quad_statem.cpp)
target_link_libraries(manager_node rm_lib)
target_link_libraries(quad rm_lib)

cs_install()
cs_export()
