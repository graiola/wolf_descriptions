cmake_minimum_required(VERSION 3.5)
project(sensors_description)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    catkin_package()
    add_definitions(-DROS)
    set(ROS_VERSION ros)
elseif(${ament_cmake_FOUND})
    ament_package()
    add_definitions(-DROS2)
    set(ROS_VERSION ros2)
endif()

find_package(camera_info_manager REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  include/${ROS_VERSION}
  ${camera_info_manager_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(RealsenseGazeboPlugin
    src/${ROS_VERSION}/RealSensePlugin.cpp
    src/${ROS_VERSION}/gazebo_ros_realsense.cpp)
target_link_libraries(RealsenseGazeboPlugin ${camera_info_manager_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS RealsenseGazeboPlugin
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

install(DIRECTORY meshes/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/meshes)

install(DIRECTORY urdfs/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/urdfs)

install(DIRECTORY launch/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch)
