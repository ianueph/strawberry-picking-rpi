cmake_minimum_required(VERSION 3.8)
project(sprpi_arm_mtc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yolo_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(sprpi_arm_mtc src/sprpi_arm_mtc.cpp)
add_executable(main_task_node src/main_task_node.cpp)
ament_target_dependencies(sprpi_arm_mtc moveit_task_constructor_core rclcpp)
ament_target_dependencies(main_task_node moveit_task_constructor_core rclcpp yolo_msgs)
target_include_directories(sprpi_arm_mtc PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_include_directories(main_task_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(sprpi_arm_mtc PUBLIC c_std_99 cxx_std_17)
target_compile_features(main_task_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS sprpi_arm_mtc main_task_node
DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# uncomment the line when a copyright and license is not present in all source files
#set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# uncomment the line when this package is not in a git repo
#set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()