cmake_minimum_required(VERSION 3.23)
project(Lidar)

set(CMAKE_CXX_STANDARD 17)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PCL 1.12 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(demo main.cpp render/render.cpp processPointClouds.cpp)
target_link_libraries (demo ${PCL_LIBRARIES})