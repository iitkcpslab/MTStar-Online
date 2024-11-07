cmake_minimum_required(VERSION 2.8.3)
project(mav_move_base)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(mav_move_base
  src/mav_move_base.cpp
)
target_link_libraries(mav_move_base ${YAML_CPP_LIBRARIES})

# ###########
# BINARIES #
# ###########
cs_add_executable(mav_move_base_node
  src/mav_move_base_node.cpp
)
target_link_libraries(mav_move_base_node mav_move_base)

##########
# EXPORT #
##########
cs_install()
cs_export()