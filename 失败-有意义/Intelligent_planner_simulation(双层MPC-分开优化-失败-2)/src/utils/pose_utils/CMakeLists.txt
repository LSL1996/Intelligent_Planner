cmake_minimum_required(VERSION 2.8.3)
project(pose_utils)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_utils
)

find_package(Armadillo REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${ARMADILLO_INCLUDE_DIRS}
    include
    )

add_library(pose_utils
   ${ARMADILLO_LIBRARIES}
   src/pose_utils.cpp)
