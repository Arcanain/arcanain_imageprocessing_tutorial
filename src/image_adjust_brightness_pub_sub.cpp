#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ImageBrightnessAdjuster : public rclcpp::Node
{
public:
  ImageBrightnessAdjuster() : Node("image_adjust_brightness_pub_sub")
  {
    this->declare_parameter<float>("brightness_factor", 1.0);  // Default brightness factor is 1 (no change)

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, std::bind(&ImageBrightnessAdjuster::image_callback, this, std::placeholders::_1));
      
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_raw_transformed", 10);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    
    RCLCPP_INFO(this->get_logger(), "Received image");

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat &cv_image = cv_ptr->image;

    // Get the brightness factor parameter
    float brightness_factor;
    this->get_parameter("brightness_factor", brightness_factor);

    // Adjust the brightness of the image
    cv_image.convertTo(cv_image, -1, brightness_factor, 0);  // Scale the brightness

    // Publish the adjusted image
    sensor_msgs::msg::Image out_msg;
    cv_bridge::CvImage(cv_ptr->header, "bgr8", cv_image).toImageMsg(out_msg);
    publisher_->publish(out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageBrightnessAdjuster>());
  rclcpp::shutdown();
  return 0;
}
