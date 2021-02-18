cmake_minimum_required(VERSION 3.5)
project(navigation_experiments_mc_bts)

set(CMAKE_BUILD_TYPE DEBUG)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(mros2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(system_modes REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  ros2_knowledge_graph
  ament_index_cpp
  nav2_msgs
  behaviortree_cpp_v3
  mros2_msgs
  std_srvs
  system_modes
)


add_library(navigation_experiments_mc_bts_navigate_to_wp_bt_node SHARED
  src/behavior_tree_nodes/NavigateToWp.cpp
)
list(APPEND plugin_libs navigation_experiments_mc_bts_navigate_to_wp_bt_node)

add_library(navigation_experiments_mc_bts_recharge_bt_node SHARED
  src/behavior_tree_nodes/Recharge.cpp
)
list(APPEND plugin_libs navigation_experiments_mc_bts_recharge_bt_node)

add_library(navigation_experiments_mc_bts_reconfigure_bt_node SHARED
  src/behavior_tree_nodes/Reconfigure.cpp
)
list(APPEND plugin_libs navigation_experiments_mc_bts_reconfigure_bt_node)

add_library(navigation_experiments_mc_bts_check_component_bt_node SHARED
  src/behavior_tree_nodes/CheckComponent.cpp
)
list(APPEND plugin_libs navigation_experiments_mc_bts_check_component_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(bt_controller src/bt_controller.cpp)
ament_target_dependencies(bt_controller ${dependencies})
target_link_libraries(bt_controller)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  bt_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
