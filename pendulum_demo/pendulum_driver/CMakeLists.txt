cmake_minimum_required(VERSION 3.8)
project(pendulum_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(pendulum_driver_node src/pendulum_driver_node.cpp)
ament_target_dependencies(pendulum_driver_node lifecycle_msgs rclcpp rclcpp_lifecycle)

install(TARGETS
  pendulum_driver_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
