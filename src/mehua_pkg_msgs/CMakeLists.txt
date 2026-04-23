cmake_minimum_required(VERSION 3.8)
project(mehua_pkg_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# === Message generation ===
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KFSDetection.msg"
  "msg/KFSDetectionArray.msg"
  DEPENDENCIES std_msgs
)

# === Export runtime dependency ===
ament_export_dependencies(rosidl_default_runtime std_msgs)

# === Testing (optional) ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()