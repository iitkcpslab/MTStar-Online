cmake_minimum_required(VERSION 2.8.3)
project(mrrtpe_utils)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############

# ###########
# BINARIES #
# ###########

##########
# EXPORT #
##########
cs_install()
cs_export()