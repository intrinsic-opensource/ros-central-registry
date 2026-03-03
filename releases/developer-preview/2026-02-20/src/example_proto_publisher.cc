// Copyright 2025 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "example/msg/example_message__typeadapter_protobuf_cpp.hpp"

using namespace std::chrono_literals;

class ExampleProtoPublisher : public rclcpp::Node {
public:
  ExampleProtoPublisher() : Node("minimal_proto_publisher") {
    publisher_ = this->create_publisher<example::msg::pb::ExampleMessage>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = example::msg::pb::ExampleMessage();
        RCLCPP_INFO(this->get_logger(), "Published protobuf message");
        this->publisher_->publish(message);
      };
    timer_ = this->create_wall_timer(1s, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<example::msg::pb::ExampleMessage>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleProtoPublisher>());
  rclcpp::shutdown();
  return 0;
}