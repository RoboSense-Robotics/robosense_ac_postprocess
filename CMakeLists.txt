cmake_minimum_required(VERSION 3.8)
project(robosense_ac_postprocess)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

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

target_link_libraries(
  postprocess_node
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(postprocess_node
  rclcpp
  sensor_msgs
  cv_bridge
  PCL
  pcl_conversions
)


install(TARGETS postprocess_node
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
 launch
 rviz2_config
 config
 DESTINATION share/${PROJECT_NAME})

ament_package()
