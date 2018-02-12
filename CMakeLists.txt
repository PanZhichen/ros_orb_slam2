cmake_minimum_required(VERSION 2.8.3)
project(ros_orb_slam2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
add_definitions(-DCOMPILEDWITHC11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
LIST(APPEND CMAKE_MODULE_PATH /home/nrsl/orb_ws/src/Lidar_in_Orb/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# catkin Build
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  cv_bridge
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    sensor_msgs
    geometry_msgs
    image_transport
    tf
    pcl_ros
)

set(ORBSLAM2_DIR "/home/nrsl/orb_ws/src/Lidar_in_Orb")
if(ORBSLAM2_DIR)
    message(STATUS "ORBSLAM2_HOME at ${OPENPOSE_DIR}")
else()
    message(FATAL_ERROR "Undefined ORBSLAM2_HOME env var.")
endif()

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}

  ${PROJECT_SOURCE_DIR}/include

  ${ORBSLAM2_DIR}/include
  ${ORBSLAM2_DIR}/
)

add_executable(${PROJECT_NAME}_mono ${PROJECT_SOURCE_DIR}/src/orb_slam2_mono.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_mono
  ${catkin_LIBRARIES}
  ${ORBSLAM2_DIR}/lib/libORB_SLAM2.so
  ${ORBSLAM2_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORBSLAM2_DIR}/Thirdparty/g2o/lib/libg2o.so

  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
)

add_executable(doPointCloud_node src/processDepthmap.cpp)
target_link_libraries(doPointCloud_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


add_executable(transPointCloud_node src/trans_vlp.cpp)
target_link_libraries(transPointCloud_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

add_executable(test_node src/test.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


