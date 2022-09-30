cmake_minimum_required(VERSION 3.10)
project(vdbfusion)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# NOTE: CHOOSE THE O2/O3 is really important for speed! check more on optimization on compile
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3")
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/MarchingCubes.cpp
        src/VDBVolume.cpp
        )
###############
# Executables #
###############

target_link_libraries(${PROJECT_NAME}
        OpenVDB::openvdb
        )

###########
# Export #
###########

cs_install()
cs_export(INCLUDE_DIRS)