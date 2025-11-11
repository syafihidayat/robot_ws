cmake_minimum_required(VERSION 3.8)
project(mehua_move)

# Compiler warning options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================================
# Find dependencies
# =====================================
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


# =====================================
# Build executable
# =====================================
add_executable(move src/move.cpp)

ament_target_dependencies(move
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
)

# =====================================
# Install
# =====================================
install(TARGETS
  move
  DESTINATION lib/${PROJECT_NAME}
)

# =====================================
# Testing (optional)
# =====================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
