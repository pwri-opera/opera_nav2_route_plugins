cmake_minimum_required(VERSION 3.8)
project(opera_nav2_route_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nav2_route REQUIRED)
find_package(opera_bt_msgs REQUIRED)

nav2_package()

include_directories(
  include
)

add_library(route_operations SHARED
  src/plugins/route_operations/opera_route_visualizer.cpp
)
target_include_directories(route_operations
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${nav2_route_INCLUDE_DIRS}
)
target_link_libraries(route_operations PUBLIC
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2_ros::tf2_ros
  nlohmann_json::nlohmann_json
  ${opera_bt_msgs_TARGETS}
)
target_link_libraries(route_operations PRIVATE
  ${lifecycle_msgs_TARGETS}
  rclcpp_components::component
  tf2::tf2
)

add_library(graph_file_loaders SHARED
  src/plugins/graph_file_loaders/opera_geojson_graph_file_loader.cpp
)
target_include_directories(graph_file_loaders
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${nav2_route_INCLUDE_DIRS}
)
target_link_libraries(graph_file_loaders PUBLIC
  nav2_core::nav2_core
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav2_msgs_TARGETS}
  nav2_util::nav2_util_core
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2_ros::tf2_ros
  nlohmann_json::nlohmann_json
  ${opera_bt_msgs_TARGETS}
)
target_link_libraries(graph_file_loaders PRIVATE
  ${lifecycle_msgs_TARGETS}
  rclcpp_components::component
  tf2::tf2
)

 
pluginlib_export_plugin_description_file(nav2_route plugins.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name} route_operations graph_file_loaders
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  visualization_msgs
  nav2_msgs
  nav_msgs
  tf2_ros
  nav2_util
  nav2_core
  nanoflann
  nlohmann_json
)
ament_export_libraries(${library_name} route_operations graph_file_loaders)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
