cmake_minimum_required(VERSION 2.8.3)
project(lego_loam_bor)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -g ")

#

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  image_transport
  rosbag_storage

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS system timer thread serialization chrono)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cloud_msgs rosbag_storage
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
	${Boost_LIBRARY_DIRS}
)

add_executable(lego_loam_bor_localise
    src/imageProjection.cpp
    src/featureAssociation.cpp
    src/mapOptmization_localise.cpp
    src/transformFusion.cpp
    src/poseInitialization.cpp
    src/main_localise.cpp)

add_dependencies(lego_loam_bor_localise ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(lego_loam_bor_localise ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)

add_executable(lego_loam_bor_create_map
    src/imageProjection.cpp
    src/featureAssociation.cpp
    src/dumpGraph.cpp
    src/mapOptmization_createMap.cpp
    src/transformFusion.cpp
    src/poseInitialization.cpp
    src/main_createMap.cpp)
    # src/main_createMap_topic.cpp)

add_dependencies(lego_loam_bor_create_map ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(lego_loam_bor_create_map ${catkin_LIBRARIES} ${PCL_LIBRARIES} gtsam)
