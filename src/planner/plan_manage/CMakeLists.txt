cmake_minimum_required(VERSION 2.8.3)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  bspline_opt
  traj_utils
  message_generation
  cv_bridge
)




# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils 
#  DEPENDS system_lib
)

include_directories(/usr/local/include/munkres)
set(MunkresLIB
        /usr/local/lib/libmunkres.a
        )

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(planner_node
  src/planner_node.cpp 
  src/topo_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(planner_node 
  ${catkin_LIBRARIES}
  )
# add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable( topo_test_node
  src/topo_test_node.cpp
  src/planner_manager.cpp
  )
target_link_libraries( topo_test_node
  ${catkin_LIBRARIES}
)

add_executable( topo_traj_node  
  src/topo_traj_node.cpp
  src/planner_manager.cpp
)
target_link_libraries( topo_traj_node
  ${catkin_LIBRARIES}
)




