cmake_minimum_required(VERSION 3.8)
project(savasan_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Main executable
add_executable(savasan_bt_node src/main.cpp
              src/nodes/haberlesme_koptu_mu.cpp
              )

target_include_directories(savasan_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(savasan_bt_node
  rclcpp
  behaviortree_cpp
  mavros_msgs
  std_msgs
)

# Install executable
install(TARGETS savasan_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config (XML files)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()