cmake_minimum_required(VERSION 2.8.3)
project(fake_battery)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(fake_battery
  src/fake_battery.cpp
)

############
# BINARIES #
############
cs_add_executable(fake_battery_node
  src/fake_battery_node.cpp
)
target_link_libraries(fake_battery_node fake_battery)

##########
# EXPORT #
##########
cs_install()
cs_export()