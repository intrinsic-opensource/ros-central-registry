#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image__typeadapter_protobuf_cpp.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_proto_publisher") {
    publisher_ = this->create_publisher<sensor_msgs::msg::pb::CompressedImage>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = sensor_msgs::msg::pb::CompressedImage();
        RCLCPP_INFO(this->get_logger(), "Published protbuf message");
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(1s, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::pb::CompressedImage>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}