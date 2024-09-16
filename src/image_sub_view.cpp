#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

class MinimalImageSubscriber : public rclcpp::Node
{
public:
  MinimalImageSubscriber(const std::string& topic_name, int width, int height) 
  : Node("opencv_image_subscriber"), target_width_(width), target_height_(height)
  {
    // Initialize subscriber with the given topic name
    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_name, 2, std::bind(&MinimalImageSubscriber::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", topic_name.c_str());
  }

  ~MinimalImageSubscriber()
  {
    // Close all OpenCV windows when the node is being destroyed
    cv::destroyAllWindows();
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Convert ROS image message to OpenCV image
      cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

      // Resize the image if target dimensions are specified
      if (target_width_ > 0 && target_height_ > 0) {
        cv::resize(cv_image, cv_image, cv::Size(target_width_, target_height_));
      }

      // Display image
      cv::imshow("Subscribed Image", cv_image);
      cv::waitKey(1); // Wait for a key press for 1 millisecond to allow UI events to process
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  int target_width_;
  int target_height_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // Default values
  std::string topic_name = "/image_raw";
  int width = 0;  // Default: don't resize
  int height = 0; // Default: don't resize

  // Check if a topic name is provided via command line
  if (argc > 1) {
    topic_name = argv[1];
  }

  // Check if width and height are provided via command line
  if (argc > 3) {
    width = std::stoi(argv[2]);
    height = std::stoi(argv[3]);
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting node with topic: %s", topic_name.c_str());
  if (width > 0 && height > 0) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Resizing image to: %d x %d", width, height);
  }

  // Create the subscriber node with the specific topic name and image size
  auto node = std::make_shared<MinimalImageSubscriber>(topic_name, width, height);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
