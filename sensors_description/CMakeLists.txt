cmake_minimum_required(VERSION 3.5)
project(sensors_description)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wmaybe-uninitialized -Wuninitialized -fPIC")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wmaybe-uninitialized -Wuninitialized -fPIC")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 nor C++17 support. Please use a different C++ compiler.")
endif()

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    #catkin_package(LIBRARIES RealsenseGazeboPlugin CATKIN_DEPENDS camera_info_manager)
    catkin_package()
    add_definitions(-DROS)
    set(ROS_VERSION ros)
elseif(${ament_cmake_FOUND})
    #ament_export_dependencies(camera_info_manager)
    ament_package()
    add_definitions(-DROS2)
    set(ROS_VERSION ros2)
endif()

find_package(sensor_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(
  include/${ROS_VERSION}
  ${sensor_msgs_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${camera_info_manager_INCLUDE_DIRS}
)

#add_library(RealsenseGazeboPlugin
#    src/${ROS_VERSION}/RealSensePlugin.cpp
#    src/${ROS_VERSION}/gazebo_ros_realsense.cpp)
#target_link_libraries(RealsenseGazeboPlugin ${image_transport_LIBRARIES} ${camera_info_manager_LIBRARIES} ${GAZEBO_LIBRARIES})
#
#install(TARGETS RealsenseGazeboPlugin
#    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

install(DIRECTORY meshes/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/meshes)

install(DIRECTORY urdfs/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/urdfs)

install(DIRECTORY launch/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch)
