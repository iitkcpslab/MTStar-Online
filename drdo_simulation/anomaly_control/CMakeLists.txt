cmake_minimum_required(VERSION 2.8.3)
project(anomaly_control)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(anomaly_control
  src/object_control.cpp
  src/anomaly_control.cpp
)
target_link_libraries(anomaly_control ${YAML_CPP_LIBRARIES})

# ###########
# BINARIES #
# ###########
cs_add_executable(anomaly_control_node
  src/anomaly_control_node.cpp
)
target_link_libraries(anomaly_control_node anomaly_control)

##########
# EXPORT #
##########
cs_install()
cs_export()