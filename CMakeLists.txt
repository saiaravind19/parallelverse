cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
cmake_policy(VERSION 3.18)
include("${CMAKE_CURRENT_SOURCE_DIR}/external/madrona/cmake/madrona_init.cmake")

project(Madrona3DExample LANGUAGES C CXX)

include(setup)
include(dependencies)

add_subdirectory(external)
# The original Madrona escape-room demo (3D, with lidar). Not used by sim2d.
option(BUILD_ESCAPE_ROOM "Build the original escape room demo" OFF)
if(BUILD_ESCAPE_ROOM)
    add_subdirectory(src)
endif()
add_subdirectory(batch_sim2d)
