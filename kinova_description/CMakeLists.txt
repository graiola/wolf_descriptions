cmake_minimum_required(VERSION 3.5)
project(kinova_description)

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    catkin_package()
elseif(${ament_cmake_FOUND})
    ament_package()
endif()

install(DIRECTORY urdf/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/urdf)

install(DIRECTORY launch/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch)

install(DIRECTORY meshes/
   DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/meshes
   FILES_MATCHING PATTERN "*.dae"
 )
