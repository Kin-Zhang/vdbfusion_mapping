cmake_minimum_required(VERSION 3.3.2 FATAL_ERROR)  # ensure that add_definition is used
project(vdbfusion_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g -fPIC")
# NOTE: CHOOSE THE O2/O3 is really important for speed! check more on optimization on compile
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Find installed Open3D, which exports Open3D::Open3D
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/open3d")
find_package(Open3D REQUIRED)
include_directories("/usr/local/include/open3d/3rdparty")
# Libigl
include(libigl)
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/cmake/libigl/include")

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/vdbfusion_mapper.cpp
        src/transform.cpp
        )

###############
# Executables #
###############

cs_add_executable(vdbfusion_mapping
        vdbfusion_mapping_node.cpp
        )

target_link_libraries(vdbfusion_mapping ${PROJECT_NAME} 
        glog::glog
        igl::core
        OpenVDB::openvdb
        Open3D::Open3D
        )

        
# =========== test  ================== #
cs_add_executable(testing
        test/test_dda.cpp
        )
target_link_libraries(testing ${PROJECT_NAME} 
        glog::glog
        OpenVDB::openvdb
        )
###########
# Export #
###########

cs_install()
cs_export()

