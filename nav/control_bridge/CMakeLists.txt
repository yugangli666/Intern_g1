cmake_minimum_required(VERSION 3.5)
project(control_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)
add_library(musen_socket STATIC
  "src/tcp/client.cpp"
  "src/tcp/server.cpp"
  "src/tcp/session.cpp"
  "src/udp/broadcaster.cpp"
  "src/udp/listener.cpp"
  "src/address.cpp"
  "src/receiver.cpp"
  "src/sender.cpp"
  "src/socket.cpp"
)

# controller_server
add_executable(controller_server
  "src/controller_server.cpp"
)
target_link_libraries(controller_server musen_socket)
ament_target_dependencies(controller_server
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# controller_client
add_executable(controller_client
  "src/controller_client.cpp"
)
target_link_libraries(controller_client musen_socket)
ament_target_dependencies(controller_client
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# message_forward
add_executable(message_forward
  "src/message_forward.cpp"
)
target_link_libraries(message_forward)
ament_target_dependencies(message_forward
  rclcpp
  geometry_msgs
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} musen_socket)
ament_export_dependencies(rclcpp std_msgs geometry_msgs nav_msgs)

# Binaries
install(TARGETS controller_server controller_server
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS controller_client controller_client
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS message_forward message_forward
  DESTINATION lib/${PROJECT_NAME}
)

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
