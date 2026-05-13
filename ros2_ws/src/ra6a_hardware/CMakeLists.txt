cmake_minimum_required(VERSION 3.8)
project(ra6a_hardware)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(ra6a_hardware SHARED
  src/ra6a_system.cpp
)
target_include_directories(ra6a_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SERIAL_INCLUDE_DIRS}
)
target_link_libraries(ra6a_hardware ${SERIAL_LIBRARIES})
ament_target_dependencies(ra6a_hardware
  rclcpp
  hardware_interface
  pluginlib
)
pluginlib_export_plugin_description_file(
  hardware_interface
  ra6a_hardware.xml
)
install(TARGETS ra6a_hardware
  DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include/
)
ament_package()
