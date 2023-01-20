cmake_minimum_required(VERSION 3.8)
project(points_inspector)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if($ENV{ROS_VERSION} EQUAL 1)
  # ROS1
  find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
  catkin_package()

  add_executable(points_inspector_node
    src/points_inspector_ros1.cpp
  )
  target_include_directories(points_inspector_node PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(points_inspector_node
    ${catkin_LIBRARIES}
  )
else()
  # ROS2
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  ament_auto_add_executable(points_inspector_node
    src/points_inspector_ros2.cpp
  )

  ament_auto_package()
endif()

