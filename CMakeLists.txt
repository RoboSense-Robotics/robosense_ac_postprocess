cmake_minimum_required(VERSION 3.8)
project(robosense_ac_postprocess)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin QUIET)  # ROS1 核心包
find_package(ament_cmake QUIET)  # ROS2 核心包

if(catkin_FOUND)
  set(ROS_VERSION "ROS1")
elseif(ament_cmake_FOUND)
  set(ROS_VERSION "ROS2")
endif()

if(ROS_VERSION STREQUAL "ROS1")
  message(STATUS "Building with ROS1")
  add_definitions(-DUSE_ROS1)  # 定义ROS1宏
  add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
  message("PROJECT_PATH: ${PROJECT_SOURCE_DIR}")
  find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    pcl_conversions
    cv_bridge
  )
  include_directories(${catkin_INCLUDE_DIRS})
  catkin_package()
elseif(ROS_VERSION STREQUAL "ROS2")
  message(STATUS "Building with ROS2")
  add_definitions(-DUSE_ROS2)  # 定义ROS2宏

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(robosense_msgs REQUIRED)


  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

add_executable(postprocess_node src/postprocess_node.cpp
          src/postprocess/camera_model.cpp
          src/postprocess/config.cpp
          src/postprocess/range_image.cpp
          src/postprocess/imu_process.cpp
          src/postprocess/postprocess_impl.cpp
)

target_include_directories(postprocess_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

if(ROS_VERSION STREQUAL "ROS1")
  target_link_libraries(postprocess_node
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    ${catkin_LIBRARIES}
  )
elseif(ROS_VERSION STREQUAL "ROS2")
  target_link_libraries(postprocess_node
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
  )
  ament_target_dependencies(postprocess_node
    rclcpp
    sensor_msgs
    cv_bridge
    PCL
    pcl_conversions
    robosense_msgs
  )
  install(TARGETS postprocess_node
    DESTINATION lib/${PROJECT_NAME})
endif()

if(ROS_VERSION STREQUAL "ROS1")
#
elseif(ROS_VERSION STREQUAL "ROS2")
  install(DIRECTORY
  launch
  rviz2_config
  config
  DESTINATION share/${PROJECT_NAME})
  ament_package()
endif()
