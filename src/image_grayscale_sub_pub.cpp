#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ImageGrayScaleConverter : public rclcpp::Node
{
public:
  ImageGrayScaleConverter() : Node("image_grayscale_sub_pub")
  {
    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, std::bind(&ImageGrayScaleConverter::image_callback, this, std::placeholders::_1));
      
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_raw_transformed", 10);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received image");

    // ROS2画像をOpenCVのcv::Matに変換する
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat &cv_image = cv_ptr->image;
    // OpenCVを使用した画像処理を行う
    // 例: 画像をグレースケールに変換
    cv::Mat gray_image;
    cv::cvtColor(cv_image, gray_image, cv::COLOR_BGR2GRAY);

    // 処理した画像をパブリッシュ（オプション）
    sensor_msgs::msg::Image ros_image;
    cv_ptr->image = gray_image;
    cv_ptr->encoding = "mono8";
    cv_ptr->toImageMsg(ros_image);
    publisher_->publish(ros_image);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageGrayScaleConverter>());
  rclcpp::shutdown();
  return 0;
}
