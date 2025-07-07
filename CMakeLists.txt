cmake_minimum_required(VERSION 3.8)
project(jetank_motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
# find_package(joint_state_publisher REQUIRED) Should only be needed for executing
# find_package(robot_state_publisher REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(serial REQUIRED) Not in ROS2
find_package(diagnostic_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

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

add_executable(robot_controller 
  src/motor/robot_controller
  src/motor/motor.cpp)

add_library(jetank_motor_lib
    src/motor/motor.cpp
)

ament_target_dependencies(robot_controller rclcpp geometry_msgs std_msgs)
ament_target_dependencies(jetank_motor_lib rclcpp)

include_directories(include)



# Install
install(TARGETS robot_controller
  DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/
  DESTINATION include/)

target_include_directories(robot_controller PRIVATE ${GPIOD_INCLUDE_DIRS})
target_link_libraries(robot_controller ${GPIOD_LIBRARIES})

target_include_directories(jetank_motor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_package()
