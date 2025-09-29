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

#include "example/msg/example_message.hpp"

using namespace std::chrono_literals;

class ExampleRosPublisher : public rclcpp::Node {
public:
  ExampleRosPublisher() : Node("example_ros_publisher"), count_(0) {
    publisher_ = this->create_publisher<example::msg::ExampleMessage>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto msg = example::msg::ExampleMessage();
        msg.message.data = "Hello, world! from C++ " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", msg.message.data.c_str());
        this->publisher_->publish(msg);
      };
    timer_ = this->create_wall_timer(1s, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<example::msg::ExampleMessage>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleRosPublisher>());
  rclcpp::shutdown();
  return 0;
}
