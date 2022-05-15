cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  nav_msgs
  plan_env
  path_searching
  quadrotor_msgs
		uav_utils
  cv_bridge
  visualization_msgs
  traj_utils

)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATKIN_DEPENDS plan_env path_searching traj_utils message_runtime
#  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

include_directories(
    include
    SYSTEM 
    third_party
  	${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  	${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS} 
)

add_executable(Intelligent_planner_node
	src/Intelligent_planner_node.cpp
  src/Intelligent_replan_fsm.cpp
  src/Intelligent_planner_manager.cpp
)

target_link_libraries(Intelligent_planner_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

