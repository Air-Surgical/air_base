cmake_minimum_required(VERSION 3.8)
project(air_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(kinova_planning_scene src/kinova_planning_scene.cpp)

target_include_directories(kinova_planning_scene PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kinova_planning_scene PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  kinova_planning_scene
  "rclcpp" 
  "moveit_ros_planning_interface"
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS kinova_planning_scene
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
