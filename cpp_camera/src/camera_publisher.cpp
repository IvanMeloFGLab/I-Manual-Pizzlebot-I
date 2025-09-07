#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher()
  : Node("camera_publisher") {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/video_source/raw", 10);

    cap_.open("/dev/video100");
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "No se pudo abrir la cámara con GStreamer.");
      rclcpp::shutdown();
      return;
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33), // ~30 FPS
      std::bind(&CameraPublisher::timer_callback, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Empezando a publicar frames.");
  }

private:
  void timer_callback() {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN(this->get_logger(), "No se pudo leer frame de la cámara.");
      return;
    }

    // Convertir cv::Mat -> sensor_msgs::msg::Image
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "camera_link";

    auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
    publisher_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
