cmake_minimum_required(VERSION 3.8)
project(jetank_motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies (only packages actually compiled/linked against; runtime
# packages such as controller_manager live in package.xml as exec_depends)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(hardware_interface REQUIRED)

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

# Motor driver library: compiled once, linked into both the executable and
# the gtest target so they exercise identical object code.
add_library(jetank_motor_lib
  src/motor/motor.cpp)

add_executable(robot_controller
  src/motor/robot_controller.cpp)

ament_target_dependencies(robot_controller
  rclcpp geometry_msgs std_msgs nav_msgs tf2 tf2_ros)
ament_target_dependencies(jetank_motor_lib rclcpp)
target_link_libraries(robot_controller jetank_motor_lib)

include_directories(include)

# ----------------------------------------------------------------------------
# ros2_control hardware interface plugin (real Feetech serial bus servos)
# ----------------------------------------------------------------------------
add_library(jetank_serial_hardware SHARED
  src/hardware/jetank_serial_hardware.cpp
  src/hardware/feetech_bus.cpp)
target_include_directories(jetank_serial_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(jetank_serial_hardware
  rclcpp rclcpp_lifecycle hardware_interface pluginlib)
# Export the plugin so pluginlib/controller_manager can discover the class.
pluginlib_export_plugin_description_file(hardware_interface jetank_hardware.xml)

# Install
install(TARGETS robot_controller
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS jetank_serial_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install headers
install(DIRECTORY include/
  DESTINATION include/)

# Install config files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

# Install hardware interface plugin description
install(FILES jetank_hardware.xml
  DESTINATION share/${PROJECT_NAME})

target_include_directories(jetank_motor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motor_header test/test_motor_header.cpp)
  target_include_directories(test_motor_header PRIVATE include)
  ament_target_dependencies(test_motor_header rclcpp)
  target_link_libraries(test_motor_header jetank_motor_lib)
endif()

ament_package()
