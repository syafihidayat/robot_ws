cmake_minimum_required(VERSION 3.8)
project(astar_pkg)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# Dependencies
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ========================
# Build node
# ========================
add_executable(astar_node src/astar.cpp)

target_include_directories(astar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(astar_node
  rclcpp
  geometry_msgs
  nav_msgs
)

target_compile_features(astar_node PUBLIC cxx_std_17)

# ========================
# Install
# ========================
install(TARGETS astar_node
  DESTINATION lib/${PROJECT_NAME}
)

# ========================
# Testing (optional)
# ========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()