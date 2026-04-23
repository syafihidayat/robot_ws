cmake_minimum_required(VERSION 3.8)
project(mehua_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(realsense2 REQUIRED)
find_package(mehua_pkg_msgs REQUIRED)

#ONNX Runtime
set(ONNXRUNTIME_DIR "/home/syafihidayat/Documents/onnxruntime-linux-x64-1.21.0")
message(STATUS "ONNXRUNTIME_DIR: ${ONNXRUNTIME_DIR}")

include_directories(
  include
  ${ONNXRUNTIME_DIR}/include
)

add_executable(mehua_cam
  src/mehua_cam.cpp
  src/utils.cpp
  src/yolov8Predictor.cpp
)

target_link_libraries(mehua_cam
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so
  realsense2::realsense2
)

ament_target_dependencies(mehua_cam
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  mehua_pkg_msgs
)

install(TARGETS mehua_cam
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

if(WIN32)
  target_link_libraries(mehua_cam "${ONNXRUNTIME_DIR}/lib/onnxruntime.lib")
elseif(UNIX)
  target_link_libraries(mehua_cam "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")
endif()

ament_package()



