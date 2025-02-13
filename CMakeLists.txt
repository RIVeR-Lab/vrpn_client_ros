cmake_minimum_required(VERSION 3.5)
project(vrpn_client_ros)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# find_package(catkin REQUIRED COMPONENTS geometry_msgs tf2_ros)
find_package(VRPN REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install headers
install(
  DIRECTORY "include/"
  DESTINATION include/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/offset.py
  scripts/relative_tracking.py
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(vrpn_client_ros
            src/vrpn_client_ros.cpp)
ament_target_dependencies(vrpn_client_ros rclcpp std_msgs tf2 tf2_ros geometry_msgs VRPN)

add_executable(vrpn_tracker_node src/vrpn_tracker_node.cpp)
target_link_libraries(vrpn_tracker_node vrpn_client_ros)
ament_target_dependencies(vrpn_tracker_node rclcpp std_msgs tf2 tf2_ros geometry_msgs VRPN)

add_executable(vrpn_client_node src/vrpn_client_node.cpp)
target_link_libraries(vrpn_client_node vrpn_client_ros)
ament_target_dependencies(vrpn_client_node rclcpp std_msgs tf2 tf2_ros geometry_msgs VRPN)

target_include_directories(vrpn_client_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_include_directories(vrpn_tracker_node
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(vrpn_client_node
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  vrpn_client_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  vrpn_tracker_node
  vrpn_client_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
