cmake_minimum_required(VERSION 2.8.3)
project(ugv_state_machine)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# add_definitions(-std=c++17)
set(CMAKE_BUILD_TYPE Release)

cs_add_library(ugv_state
    src/ugv_state/ugv_state.cpp
    src/ugv_state/move_base.cpp
)
target_link_libraries(ugv_state ${catkin_LIBRARIES})

cs_add_library(ugv_behaviours
    src/behaviours/follow_anomaly.cpp
    src/behaviours/goal_reached.cpp
    src/behaviours/reach_goal.cpp
    src/behaviours/reach_idle.cpp
    src/behaviours/startup.cpp
)
target_link_libraries(ugv_behaviours ugv_state)

cs_add_library(ugv_state_machine_lib
    src/ugv_state_machine_base.cpp
    src/ugv_state_machine.cpp
)
target_link_libraries(ugv_state_machine_lib ugv_behaviours ${catkin_LIBRARIES})

cs_add_executable(ugv_state_machine_node src/ugv_state_machine_node.cpp)
target_link_libraries(ugv_state_machine_node ugv_state_machine_lib)

cs_install()
cs_export()
