// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <example/msg/example_message.h>
#include <stdio.h>

void subscription_callback(const void * msgin) {
    const example__msg__ExampleMessage * msg = (const example__msg__ExampleMessage *)msgin;
    
    // Accessing the nested string: msg->message (std_msgs/String) -> data (RosString) -> data (char*)
    if (msg->message.data.data != NULL) {
        printf("I heard: %s\n", msg->message.data.data);
    }
}

int main() {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "example_ros_subscriber_c", "", &support);

    rcl_subscription_t subscriber;
    rclc_subscription_init_default(
        &subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(example, msg, ExampleMessage), 
        "example_topic");

    // Setup Executor (handles 1 communication handle)
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    
    example__msg__ExampleMessage msg;
    example__msg__ExampleMessage__init(&msg);
    
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

    printf("Subscriber node started. Waiting for messages...\n");
    while(rcl_context_is_valid(&support.context)) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Cleanup
    example__msg__ExampleMessage__fini(&msg);
    rclc_executor_fini(&executor);
    rcl_subscription_fini(&subscriber, &node);
    rcl_node_fini(&node);
    return 0;
}