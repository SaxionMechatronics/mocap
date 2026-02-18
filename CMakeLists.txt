cmake_minimum_required(VERSION 3.5)

project(mocap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Select NatNet SDK directory based on target architecture.
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(NATNET_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/NatNetARM64/)
  set(NATNET_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/NatNetARM64/lib/libNatNet.so)
else()
  set(NATNET_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/NatNetAMD64/)
  set(NATNET_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vendor/NatNetAMD64/lib/libNatNet.so)
endif()

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${NATNET_SDK_DIR}/include)

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
