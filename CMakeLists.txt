cmake_minimum_required(VERSION 3.8)
project(adog_legged_hardware_interface)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  adog_legged_hardware_interface
  SHARED
  src/adog_legged_hardware.cpp
  src/usb_hardware.cpp
  src/serialib.cpp
)
target_include_directories(
  adog_legged_hardware_interface
  PUBLIC
  include
)
ament_target_dependencies(
  adog_legged_hardware_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
# prevent pluginlib from using boost
target_compile_definitions(adog_legged_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface adog_legged_hardware_interface.xml)

install(
  TARGETS
  adog_legged_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)

  # ament_add_gmock(test_adog_legged_hardware test/test_adog_legged_hardware.cpp)
  # target_include_directories(test_adog_legged_hardware PRIVATE include)
  # ament_target_dependencies(
  #   test_adog_legged_hardware
  #   hardware_interface
  #   pluginlib
  #   ros2_control_test_assets
  # )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  adog_legged_hardware_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
