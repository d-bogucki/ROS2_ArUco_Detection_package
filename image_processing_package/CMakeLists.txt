cmake_minimum_required(VERSION 3.8)
project(image_processing_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(opencv_subscriber_node src/opencv_subscriber_node.cpp)
target_include_directories(opencv_subscriber_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(opencv_subscriber_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  opencv_subscriber_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "OpenCV"
)

add_executable(esp32_cam_calibration src/esp32_cam_calibration.cpp)
target_include_directories(esp32_cam_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(esp32_cam_calibration PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  esp32_cam_calibration
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "OpenCV"
)

add_executable(aruco_detection_and_pose_estimation src/aruco_detection_and_pose_estimation.cpp)
target_include_directories(aruco_detection_and_pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(aruco_detection_and_pose_estimation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  aruco_detection_and_pose_estimation
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
)

install(TARGETS 
opencv_subscriber_node
esp32_cam_calibration
aruco_detection_and_pose_estimation
DESTINATION lib/${PROJECT_NAME})

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
