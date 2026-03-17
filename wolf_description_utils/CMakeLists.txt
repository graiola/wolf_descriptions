cmake_minimum_required(VERSION 3.5)
project(wolf_description_utils)

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    set(ROS_VERSION "ros")
    catkin_package()
elseif(${ament_cmake_FOUND})
    set(ROS_VERSION "ros2")
    find_package(ignition-gazebo6 REQUIRED)
    find_package(ignition-plugin1 REQUIRED COMPONENTS register)

    add_library(wolf_joint_position_reset_system SHARED
      src/joint_position_reset_system.cpp)
    target_compile_features(wolf_joint_position_reset_system PUBLIC cxx_std_17)
    target_link_libraries(wolf_joint_position_reset_system
      ignition-gazebo6::ignition-gazebo6
      ignition-plugin1::register)

    install(TARGETS wolf_joint_position_reset_system
      LIBRARY DESTINATION lib
      ARCHIVE DESTINATION lib
      RUNTIME DESTINATION bin)
endif()

install(DIRECTORY launch/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/launch)

install(DIRECTORY urdfs/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/urdfs)

install(PROGRAMS scripts/${ROS_VERSION}/go0 scripts/ros2/description_loader scripts/create_urdf_model.sh scripts/create_gazebo_model.sh
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})

if(${ament_cmake_FOUND})
  install(PROGRAMS scripts/ros2/ignition_world
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME})
  ament_package()
endif()
