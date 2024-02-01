cmake_minimum_required(VERSION 3.8)
project(tortoisebot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES geometry_msgs
)
include_directories(include)



add_executable(action_server src/tortoisebot_action_server.cpp)
ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
)
rosidl_target_interfaces(action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS
	action_server
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)



add_library(action_server_gtest src/tortoisebot_action_server.cpp)
ament_target_dependencies(action_server_gtest 
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
)

rosidl_target_interfaces(action_server_gtest
      ${PROJECT_NAME} "rosidl_typesupport_cpp")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
      test/waypoints_test_v1.cpp
  )
  ament_add_gtest(${PROJECT_NAME}_wt ${TESTFILES})
  target_link_libraries(${PROJECT_NAME}_wt action_server_gtest)
  ament_target_dependencies(${PROJECT_NAME}_wt 
	rclcpp
	rclcpp_action
	std_msgs
	geometry_msgs
	nav_msgs
	tf2
  )
  install(TARGETS
        ${PROJECT_NAME}_wt
        DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()