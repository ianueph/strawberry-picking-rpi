cmake_minimum_required(VERSION 3.16)
project(sprpi_arm_hwi LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  sprpi_arm_hwi
  SHARED
  hardware/sprpi_arm_hwi.cpp
)
target_link_libraries(sprpi_arm_hwi PRIVATE /lib/aarch64-linux-gnu/libserial.so)
target_compile_features(sprpi_arm_hwi PUBLIC cxx_std_17)
target_include_directories(sprpi_arm_hwi PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/sprpi_arm_hwi>
)
ament_target_dependencies(
  sprpi_arm_hwi PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface sprpi_arm_hwi.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/sprpi_arm_hwi
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/sprpi_arm_hwi
)
install(TARGETS sprpi_arm_hwi
  EXPORT export_sprpi_arm_hwi
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

#  ament_add_pytest_test(example_1_urdf_xacro test/test_urdf_xacro.py)
#  ament_add_pytest_test(view_example_1_launch test/test_view_robot_launch.py)
#  ament_add_pytest_test(run_example_1_launch test/test_rrbot_launch.py)
#  ament_add_pytest_test(run_example_1_launch_cli_direct test/test_rrbot_launch_cli_direct.py)
endif()


## EXPORTS
ament_export_targets(export_sprpi_arm_hwi HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
