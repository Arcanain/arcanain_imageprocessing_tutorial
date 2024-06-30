#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ImageContrastAdjuster : public rclcpp::Node
{
public:
  ImageContrastAdjuster() : Node("image_adjust_contrast_pub_sub")
  {
    // コントラスト調整のためのスケールファクターをデフォルトは1.0 (変更なし)
    this->declare_parameter<float>("contrast_factor", 1.0);

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, std::bind(&ImageContrastAdjuster::image_callback, this, std::placeholders::_1));
      
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

    // コントラスト係数の取得
    float contrast_factor;
    this->get_parameter("contrast_factor", contrast_factor);

    // 画像のコントラストを調整
    cv_image.convertTo(cv_image, -1, contrast_factor, 0);  // スケールファクターでコントラストを調整

    // 処理した画像をパブリッシュ
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
  rclcpp::spin(std::make_shared<ImageContrastAdjuster>());
  rclcpp::shutdown();
  return 0;
}
