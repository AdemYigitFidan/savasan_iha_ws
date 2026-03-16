cmake_minimum_required(VERSION 3.8)
project(target_scorer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json REQUIRED)

# Custom mesajları oluştur
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScoredTarget.msg"
  "msg/ScoredTargetArray.msg"
  DEPENDENCIES std_msgs
)

# Target scorer node
add_executable(target_scorer_node src/target_scorer_node.cpp)

target_include_directories(target_scorer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(target_scorer_node
  rclcpp
  std_msgs
  geometry_msgs
  mavros_msgs
  sensor_msgs
)

target_link_libraries(target_scorer_node
  nlohmann_json::nlohmann_json
)

# Custom mesajları kullanabilmesi için
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(target_scorer_node "${cpp_typesupport_target}")

install(TARGETS target_scorer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
