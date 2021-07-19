cmake_minimum_required(VERSION 2.8.3)
project(ltslam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS 
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime 
  message_generation
)

# include directories
include_directories(
	include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
  include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###########
## Build ##
###########

#ltslam 
add_executable(${PROJECT_NAME}_ltslam src/main.cpp
  src/utility.cpp 
  src/RosParamServer.cpp
  src/Session.cpp
  src/LTslam.cpp
  src/Scancontext.cpp
)

add_dependencies(${PROJECT_NAME}_ltslam ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME}_ltslam PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_ltslam 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
  stdc++fs
)


