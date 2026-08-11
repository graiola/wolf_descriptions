cmake_minimum_required(VERSION 3.5)
project(sensors_description)

# ROS
find_package(catkin QUIET)
find_package(ament_cmake QUIET)

if(${catkin_FOUND})
    catkin_package()
elseif(${ament_cmake_FOUND})
    if(BUILD_TESTING)
      find_package(ament_cmake_pytest REQUIRED)
      ament_add_pytest_test(
        test_depth_camera_contract
        test/test_depth_camera_contract.py
        TIMEOUT 30)
    endif()
    ament_package()

endif()

install(DIRECTORY urdfs/
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/urdfs)
