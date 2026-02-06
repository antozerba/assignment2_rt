cmake_minimum_required(VERSION 3.8)
project(assignment2_rt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interface REQUIRED)
find_package(sensor_msgs REQUIRED)


add_executable(ui src/ui_node.cpp)
add_executable(controller src/distance_controller.cpp)
ament_target_dependencies(ui rclcpp std_msgs geometry_msgs custom_interface)
ament_target_dependencies(controller rclcpp geometry_msgs std_msgs custom_interface sensor_msgs)

install(TARGETS
  ui 
  controller
  DESTINATION lib/${PROJECT_NAME}
)





ament_package()
