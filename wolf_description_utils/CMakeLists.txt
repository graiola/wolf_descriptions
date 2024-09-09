cmake_minimum_required(VERSION 3.5)
project(wolf_description_utils)

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    catkin_package()
elseif(${ament_cmake_FOUND})
    ament_package()
endif()

install(DIRECTORY launch/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch)

install(DIRECTORY urdfs/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/urdfs)

install(PROGRAMS scripts/go0 scripts/create_urdf_model.sh scripts/create_gazebo_model.sh
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
