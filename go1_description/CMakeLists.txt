cmake_minimum_required(VERSION 2.8.3)
project(go1_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY robots/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY urdfs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdfs
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  PATTERN ".svn" EXCLUDE)
