cmake_minimum_required(VERSION 3.5)

project(mocap)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

set(NATNET_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/NatNetAMD64/) 
set(NATNET_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/NatNetAMD64/lib/libNatNet.so)

set(my_include_directories
  include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${NATNET_SDK_DIR}/include
)

include_directories(
  include
  ${my_include_directories}
)

add_executable(natnet_ros2_node
  src/natnet_ros2.cpp 
)

ament_target_dependencies(natnet_ros2_node rclcpp geometry_msgs)
target_link_libraries(natnet_ros2_node
  ${NATNET_LIBRARY_DIR}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(TARGETS
natnet_ros2_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
natnet_ros2_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
