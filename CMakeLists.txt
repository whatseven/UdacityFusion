cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

file(GLOB  SRC src/*.cpp src/*.h src/render/*.cpp src/render/*.h src/sensor/*.h)
#file(GLOB SRC src/quiz/cluster/cluster.cpp src/quiz/cluster/kdtree.h src/render/*.cpp src/render/*.h)

add_executable (environment ${SRC})
target_link_libraries (environment ${PCL_LIBRARIES})




