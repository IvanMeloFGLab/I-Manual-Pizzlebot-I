cmake_minimum_required(VERSION 3.10)
project(cpp_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Asegurar compatibilidad gnu++2a (C++20), Jazzy soporta C++17+ y C++20.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(HailoRT REQUIRED)

add_executable(camera_pub src/camera_publisher.cpp)
ament_target_dependencies(camera_pub rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(yolo11 src/yolo.cpp)
ament_target_dependencies(yolo11 rclcpp sensor_msgs cv_bridge OpenCV)
target_link_libraries(yolo11 HailoRT::libhailort Threads::Threads)

install(TARGETS
  camera_pub
  yolo11
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY yolos
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hef"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
