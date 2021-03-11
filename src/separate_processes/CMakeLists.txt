cmake_minimum_required(VERSION 3.5)
project(separate_processes)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ping_pong_interfaces REQUIRED)

set(3RD_PARTY_INCLUDES "../3rdparty/")

add_executable(start_node src/start_node.cpp)
ament_target_dependencies(start_node rclcpp std_msgs ping_pong_interfaces)
target_include_directories(start_node PRIVATE include/separate_processes/ ${3RD_PARTY_INCLUDES})

add_executable(intermediate_node src/intermediate_node.cpp)
ament_target_dependencies(intermediate_node rclcpp std_msgs ping_pong_interfaces)
target_include_directories(intermediate_node PRIVATE include/separate_processes/ ${3RD_PARTY_INCLUDES})

add_executable(end_node src/end_node.cpp)
ament_target_dependencies(end_node rclcpp std_msgs ping_pong_interfaces)
target_include_directories(end_node PRIVATE include/separate_processes/ ${3RD_PARTY_INCLUDES})

add_executable(all_nodes_same_context src/all_nodes_same_context.cpp)
ament_target_dependencies(all_nodes_same_context rclcpp std_msgs ping_pong_interfaces)
target_include_directories(all_nodes_same_context PRIVATE include/separate_processes/ ${3RD_PARTY_INCLUDES})

add_executable(all_nodes_separate_context src/all_nodes_separate_context.cpp)
ament_target_dependencies(all_nodes_separate_context rclcpp std_msgs ping_pong_interfaces)
target_include_directories(all_nodes_separate_context PRIVATE include/separate_processes/ ${3RD_PARTY_INCLUDES})

install(TARGETS
    start_node
    intermediate_node
    end_node
    all_nodes_same_context
    all_nodes_separate_context
    DESTINATION lib/${PROJECT_NAME})
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
