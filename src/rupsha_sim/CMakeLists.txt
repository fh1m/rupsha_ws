cmake_minimum_required(VERSION 3.8)
project(rupsha_sim)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch models worlds scenarios
  DESTINATION share/${PROJECT_NAME})

# GZ_SIM_RESOURCE_PATH so `model://rupsha_skid` and the posts resolve after sourcing.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
