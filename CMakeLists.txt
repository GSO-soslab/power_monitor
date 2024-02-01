cmake_minimum_required(VERSION 3.8)
project(power_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mvp_msgs REQUIRED)

include_directories(
  include
)

set(ros_dependencies
  rclcpp
  std_msgs
  mvp_msgs
)

# build
add_executable(power_monitor_node 
  src/ros_node.cpp
  include/power_monitor/power_monitor.cpp
  include/power_monitor/MCP3424.cpp
)
# target_link_libraries(power_monitor_node 
#   ${Boost_LIBRARIES} what kind of lib need link here ???
# )
ament_target_dependencies(power_monitor_node ${ros_dependencies})

install(TARGETS power_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
