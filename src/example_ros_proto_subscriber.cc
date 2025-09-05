#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image__typeadapter_protobuf_cpp.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    auto topic_callback =
      [this](const sensor_msgs::msg::pb::CompressedImage & msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Received protobuf message");
      };
    subscription_ =
      this->create_subscription<sensor_msgs::msg::pb::CompressedImage>("topic", 10, topic_callback);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::pb::CompressedImage>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}