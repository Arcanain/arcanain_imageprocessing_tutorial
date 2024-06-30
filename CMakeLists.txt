cmake_minimum_required(VERSION 3.8)
project(arcanain_imageprocessing_tutorial)

#list(APPEND CMAKE_PREFIX_PATH "$ENV{AMENT_PREFIX_PATH}")

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
find_package(Eigen3 REQUIRED)
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


add_executable(random_image_publisher src/generate_random_image_pub.cpp)
ament_target_dependencies(random_image_publisher rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

add_executable(image_subscriber src/image_sub_view.cpp)
ament_target_dependencies(image_subscriber rclcpp cv_bridge image_transport sensor_msgs OpenCV)

add_executable(image_resize_sub_pub src/image_resize_sub_pub.cpp)
ament_target_dependencies(image_resize_sub_pub rclcpp cv_bridge image_transport sensor_msgs OpenCV)

add_executable(image_grayscale_sub_pub src/image_grayscale_sub_pub.cpp)
ament_target_dependencies(image_grayscale_sub_pub rclcpp cv_bridge image_transport sensor_msgs OpenCV)

add_executable(image_adjust_brightness_pub_sub src/image_adjust_brightness_pub_sub.cpp)
ament_target_dependencies(image_adjust_brightness_pub_sub rclcpp cv_bridge image_transport sensor_msgs OpenCV)

add_executable(image_adjust_contrast_pub_sub src/image_adjust_contrast_pub_sub.cpp)
ament_target_dependencies(image_adjust_contrast_pub_sub rclcpp cv_bridge image_transport sensor_msgs OpenCV)

add_executable(image_rotate_pub_sub src/image_rotate_pub_sub.cpp)
ament_target_dependencies(image_rotate_pub_sub rclcpp cv_bridge image_transport sensor_msgs OpenCV)

install(TARGETS
  random_image_publisher
  image_subscriber
  image_resize_sub_pub
  image_grayscale_sub_pub
  image_adjust_brightness_pub_sub
  image_adjust_contrast_pub_sub
  image_rotate_pub_sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()