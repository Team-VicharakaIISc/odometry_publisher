cmake_minimum_required(VERSION 3.8)
project(odometry_publisher)

# Add compile options for better warnings and debugging
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add the executable
add_executable(odometry_publisher src/odometry_publisher.cpp)

# Link the target with dependencies
ament_target_dependencies(
  odometry_publisher
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  nav_msgs
)

# Install the executable
install(TARGETS odometry_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Uncomment to enforce copyright checks
  # set(ament_cmake_copyright_FOUND TRUE)
  
  # Uncomment to enforce cpplint checks
  # set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# Package definition
ament_package()
