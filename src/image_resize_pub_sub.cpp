#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Dense>

class ImageResizer : public rclcpp::Node
{
public:
  ImageResizer() : Node("image_resize_pub_sub")
  {
    this->declare_parameter<int>("width", 0);  // Default width
    this->declare_parameter<int>("height", 0); // Default height

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, std::bind(&ImageResizer::image_callback, this, std::placeholders::_1));
      
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

    int new_width, new_height;
    this->get_parameter("width", new_width);
    this->get_parameter("height", new_height);

    // 画像のリサイズ処理
    cv::Mat resized_image;
    if (new_width > 0 && new_height > 0) {
        cv::resize(cv_image, resized_image, cv::Size(new_width, new_height));
    } else {
        resized_image = cv_image;  // パラメータが設定されていない場合は元のサイズを保持
    }


    // 処理した画像をパブリッシュ（オプション）
    // 処理した画像をパブリッシュ（オプション）
    sensor_msgs::msg::Image ros_image;
    cv_bridge::CvImage(cv_ptr->header, "bgr8", resized_image).toImageMsg(ros_image);
    publisher_->publish(ros_image);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageResizer>());
  rclcpp::shutdown();
  return 0;
}