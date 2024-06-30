# Arcanain Image Processing Tutorial

## Description
This package provides a set of ROS 2 nodes for various image processing tasks such as resizing, converting to grayscale, adjusting brightness and contrast, and rotating images. Each node subscribes to image topics, processes the images, and publishes the result to another topic.

## Setup

### Dependencies
- ROS 2 (Foxy or newer recommended)
- OpenCV 4.x
- cv_bridge
- image_transport

### Building the Package
1. Clone the repository into your ROS 2 workspace (e.g., `~/dev_ws/src/`).
2. Navigate to the root of your workspace (e.g., `~/dev_ws`).
3. Build the workspace using the following command:
   ```
   colcon build --packages-select arcanain_imageprocessing_tutorial
   ```
4. Source the setup file to include the workspace in your ROS 2 environment:
   ```
   source install/setup.bash
   ```

## Nodes Description and Usage

### Random Image Publisher
This node generates random images and publishes them.
- **Topic Published**: `/random_image` (sensor_msgs/Image)
- **Command to Run**:
  ```
  ros2 run arcanain_imageprocessing_tutorial random_image_publisher
  ```

### Image Subscriber (Sub View)
This node subscribes to an image topic and displays the images using OpenCV.
- **Topic Subscribed**: `/processed_image` (sensor_msgs/Image)
- **Command to Run**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_subscriber
  ```

### Image Resize Publisher-Subscriber
This node resizes images to half their original size.
- **Topic Subscribed**: `/image` (sensor_msgs/Image)
- **Topic Published**: `/resized_image` (sensor_msgs/Image)
- **Command to Run**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_resize_pub_sub
  ```

### Image Grayscale Publisher-Subscriber
Converts color images to grayscale.
- **Topic Subscribed**: `/image` (sensor_msgs/Image)
- **Topic Published**: `/grayscale_image` (sensor_msgs/Image)
- **Command to Run**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_grayscale_pub_sub
  ```

### Image Adjust Brightness Publisher-Subscriber
Adjusts the brightness of images.
- **Topic Subscribed**: `/image` (sensor_msgs/Image)
- **Topic Published**: `/brightness_adjusted_image` (sensor_msgs/Image)
- **Command to Run**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_adjust_brightness_pub_sub
  ```

### Image Adjust Contrast Publisher-Subscriber
Adjusts the contrast of images.
- **Topic Subscribed**: `/image` (sensor_msgs/Image)
- **Topic Published**: `/contrast_adjusted_image` (sensor_msgs/Image)
- **Command to Run**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_adjust_contrast_pub_sub
  ```

### Image Rotate Publisher-Subscriber
Rotates images by 90 degrees.
- **Topic Subscribed**: `/image` (sensor_msgs/Image)
- **Topic Published**: `/rotated_image` (sensor_msgs/Image)
- **Command to Run**:
  ```
  ros2 run arcanain_imageprocessing_tutorial image_rotate_pub_sub
  ```

## Running the Nodes
To run any of the above nodes, open a new terminal, source your ROS 2 environment, and use the corresponding command as shown in the usage section for each node. Ensure that the topics correspond to your specific setup or modify the topic names in the source code as needed.

