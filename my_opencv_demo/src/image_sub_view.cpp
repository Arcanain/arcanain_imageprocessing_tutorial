#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class MinimalImageSubscriber : public rclcpp::Node
{
public:
  MinimalImageSubscriber() : Node("opencv_image_subscriber")
  {
    // Initialize subscriber
    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "random_image", 10, std::bind(&MinimalImageSubscriber::image_callback, this, std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Convert ROS image message to OpenCV image
      cv::Mat cv_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

      // Display image
      cv::imshow("Subscribed Image", cv_image);
      cv::waitKey(1); // Wait for a key press for 1 millisecond to allow UI events to process
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalImageSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
