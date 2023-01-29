cmake_minimum_required(VERSION 2.8.3)
project(time_slider_controller)

find_package(gazebo REQUIRED)
# add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_filters
  message_generation
  robot_state_publisher
  eigen_conversions
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

add_message_files(
  FILES
  FloatArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime 
  DEPENDS pinocchio 
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${ros_lib_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

include_directories(
  "/usr/include/eigen3/"
  "/usr/local/include"
  "/opt/openrobots/include/"
) 

 # message(IMPORTANT MESSAGE "${pinocchio_CFLAGS_OTHER}")

FILE(GLOB foot_folder_source ./src/osc_foot.cc ./src/slider_controller_foot.cc ./src/gen_trajectory.cc)
FILE(GLOB foot_folder_header ./src/osc_foot.h ./src/slider_controller_foot.h ./src/gen_trajectory.h ./src/define.h ./src/LIPM.h)
FILE(GLOB folder_source ./src/osc.cc ./src/slider_controller.cc ./src/gen_trajectory.cc)
FILE(GLOB folder_header ./src/osc.h ./src/slider_controller.h ./src/gen_trajectory.h ./src/define.h ./src/LIPM.h)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBOOST_MATH_DISABLE_FLOAT128")
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread -fext-numeric-literals")
SET(GCC_COVERAGE_COMPILE_FLAGS " -O3 -std=c++11") # -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR -DBOOST_MPL_LIMIT_LIST_SIZE=30 -DPINOCCHIO_WITH_URDFDOM -std=c++11 -lprofilingS -p -lp -g -p -O3 -fopenmp
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

LINK_LIBRARIES(
boost_system
urdfdom_model
pinocchio
)

add_executable(slider_controller src/slider_controller.cc ${folder_source} ${foot_folder_header})
target_link_libraries(slider_controller PRIVATE libqpOASES.so ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${LIBRARIES}) 

add_executable(slider_controller_foot src/slider_controller_foot.cc ${foot_folder_source} ${folder_header})
target_link_libraries(slider_controller_foot PRIVATE libqpOASES.so ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${LIBRARIES}) 

add_executable(footstep_planner_sequential src/footstep_planner_gd.cpp)
target_link_libraries(footstep_planner_sequential ${catkin_LIBRARIES} ${LIBRARIES})

add_executable(footstep_planner_sequential_GN src/footstep_planner_BFGS.cpp)
target_link_libraries(footstep_planner_sequential_GN ${catkin_LIBRARIES} ${LIBRARIES})
