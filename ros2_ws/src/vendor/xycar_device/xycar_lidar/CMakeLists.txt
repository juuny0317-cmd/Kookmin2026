# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 3.5)
project(xycar_lidar C CXX)

##################ros2#############################################
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################find package#####################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

############## YDLIDAR SDK START#####################################
# Build the archived SDK from source on the current architecture.  The ASUS
# system-wide library was amd64 and must never be reused on Jetson arm64.
set(BUILD_EXAMPLES OFF CACHE BOOL "Do not build YDLidar examples" FORCE)
set(BUILD_CSHARP OFF CACHE BOOL "Do not build YDLidar C# bindings" FORCE)
set(BUILD_TEST OFF CACHE BOOL "Do not build YDLidar SDK tests" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Link YDLidar SDK statically" FORCE)
add_subdirectory(YDLidar-SDK ydlidar_sdk_build EXCLUDE_FROM_ALL)
############## YDLIDAR SDK END#####################################

#Include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/YDLidar-SDK
  ${PROJECT_SOURCE_DIR}/YDLidar-SDK/src
  ${CMAKE_CURRENT_BINARY_DIR}/ydlidar_sdk_build)

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp)

#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_node
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
    )

target_link_libraries(${PROJECT_NAME}_node
    ydlidar_sdk)

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_client
    src/${PROJECT_NAME}_client.cpp)

#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_client
    "rclcpp"
    "sensor_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "std_srvs"
    )

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
install(TARGETS
    ${PROJECT_NAME}_node ${PROJECT_NAME}_client
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params startup config
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
