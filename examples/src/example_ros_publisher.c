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
#include <example/msg/example_message.h>
#include <rosidl_runtime_c/string_functions.h>
#include <stdio.h>
#include <unistd.h>

int main() {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "example_ros_publisher_c", "", &support);

    rcl_publisher_t publisher;
    rclc_publisher_init_default(
        &publisher, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(example, msg, ExampleMessage), 
        "example_topic");

    // Initialize the top-level message
    example__msg__ExampleMessage msg;
    example__msg__ExampleMessage__init(&msg);
    
    int count = 0;
    while(rcl_context_is_valid(&support.context)) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Hello, world! from C %d", count++);
        
        // Path: msg (ExampleMessage) -> message (std_msgs/String) -> data (RosString)
        rosidl_runtime_c__String__assign(&msg.message.data, buffer);

        rcl_publish(&publisher, &msg, NULL);
        printf("Published: %s\n", msg.message.data.data);
        
        sleep(1);
    }

    // This frees the internal memory for the nested string AND the 4 images
    example__msg__ExampleMessage__fini(&msg);
    rcl_publisher_fini(&publisher, &node);
    rcl_node_fini(&node);
    return 0;
}
