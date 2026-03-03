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

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "example/msg/example_message__typeadapter_protobuf_cpp.hpp"

class ExampleProtoSubscriber : public rclcpp::Node
{
public:
  ExampleProtoSubscriber(): Node("example_proto_subscriber")
  {
    auto topic_callback =
      [this](const example::msg::pb::ExampleMessage & msg) -> void {
        RCLCPP_INFO(this->get_logger(), "Received protobuf message");
      };
    subscription_ =
      this->create_subscription<example::msg::pb::ExampleMessage>("topic", 10, topic_callback);
  }

private:
  rclcpp::Subscription<example::msg::pb::ExampleMessage>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExampleProtoSubscriber>());
  rclcpp::shutdown();
  return 0;
}