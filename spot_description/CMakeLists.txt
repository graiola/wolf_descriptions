cmake_minimum_required(VERSION 3.5)
project(spot_description)

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    catkin_package()
elseif(${ament_cmake_FOUND})
    ament_package()
endif()

install(DIRECTORY meshes/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/meshes)

install(DIRECTORY robots/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/robots)

install(DIRECTORY urdfs/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/urdfs)

install(DIRECTORY params/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/params)


