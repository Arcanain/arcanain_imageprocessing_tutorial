#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ImageRotator : public rclcpp::Node
{
public:
  ImageRotator() : Node("image_rotate_pub_sub")
  {
    this->declare_parameter<float>("angle", 0.0);   // 角度（度数法）
    this->declare_parameter<int>("center_x", -1);   // 回転の中心 x 座標
    this->declare_parameter<int>("center_y", -1);   // 回転の中心 y 座標

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, std::bind(&ImageRotator::image_callback, this, std::placeholders::_1));
      
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

    float angle;
    int center_x, center_y;
    this->get_parameter("angle", angle);
    this->get_parameter("center_x", center_x);
    this->get_parameter("center_y", center_y);

    cv::Mat &cv_image = cv_ptr->image;
    cv::Mat rotated_image;
    cv::Point2f center(cv_image.cols/2.0F, cv_image.rows/2.0F);
    if (center_x != -1 && center_y != -1) {
        center = cv::Point2f(center_x, center_y);
    }

    cv::Mat rotation_matrix = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::warpAffine(cv_image, rotated_image, rotation_matrix, cv_image.size());

    sensor_msgs::msg::Image out_msg;
    cv_bridge::CvImage(cv_ptr->header, "bgr8", rotated_image).toImageMsg(out_msg);
    publisher_->publish(out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageRotator>());
  rclcpp::shutdown();
  return 0;
}
