cmake_minimum_required(VERSION 3.8)
project(roa_system_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(roa_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# add_executable(roa_system_manager
#   src/system_manager_node.cpp
# )
# ament_target_dependencies(roa_system_manager
#   std_msgs
#   rclcpp
#   rclcpp_lifecycle
#   lifecycle_msgs
#   roa_interfaces
# )

# target_include_directories(roa_system_manager PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# install(TARGETS
#   roa_system_manager
#   RUNTIME DESTINATION lib/${PROJECT_NAME}
# )
add_executable(system_manager_node
  src/system_manager_node.cpp
)

ament_target_dependencies(system_manager_node
  std_msgs
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  roa_interfaces
)

target_include_directories(system_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  system_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
