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

#include <stdio.h>
#include <example/msg/example_message.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

// these data structures for the publisher and subscriber are global, so that
// they can be configured in main() and can be used in the corresponding callback.
rcl_publisher_t my_pub;
example__msg__ExampleMessage pub_msg;
example__msg__ExampleMessage sub_msg;

/***************************** CALLBACKS ***********************************/

void my_subscriber_callback(const void * msgin)
{
  const example__msg__ExampleMessage * msg = (const example__msg__ExampleMessage *)msgin;
  if (msg == NULL) {
    printf("Callback: msg NULL\n");
  } else {
    printf("Callback: I heard: %s\n", msg->message.data.data);
  }
}

void my_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  rcl_ret_t rc;
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //printf("Timer: time since last call %d\n", (int) last_call_time);
    rc = rcl_publish(&my_pub, &pub_msg, NULL);
    if (rc == RCL_RET_OK) {
      printf("Published message %s\n", pub_msg.message.data.data);
    } else {
      printf("timer_callback: Error publishing message %s\n", pub_msg.message.data.data);
    }
  } else {
    printf("timer_callback Error: timer parameter is NULL\n");
  }
}

/******************** MAIN PROGRAM ****************************************/
int main(int argc, const char * argv[])
{
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_ret_t rc;

  // create init_options
  rc = rclc_support_init(&support, argc, argv, &allocator);
  if (rc != RCL_RET_OK) {
    printf("Error rclc_support_init.\n");
    return -1;
  }

  // create rcl_node
  rcl_node_t my_node = rcl_get_zero_initialized_node();
  rc = rclc_node_init_default(&my_node, "example_ros_publisher", "example", &support);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_node_init_default\n");
    return -1;
  }

  // create a publisher to publish topic 'topic' with type example::msg::ExampleMessage
  // my_pub is global, so that the timer callback can access this publisher.
  const char * topic_name = "topic";
  const rosidl_message_type_support_t * my_type_support =
    ROSIDL_GET_MSG_TYPE_SUPPORT(example, msg, ExampleMessage);
  rc = rclc_publisher_init_default(
    &my_pub,
    &my_node,
    my_type_support,
    topic_name);
  if (RCL_RET_OK != rc) {
    printf("Error in rclc_publisher_init_default %s.\n", topic_name);
    return -1;
  }

  // Create a timer to publish the message.
  rcl_timer_t my_timer = rcl_get_zero_initialized_timer();
  const unsigned int timer_timeout = 1000;
  rc = rclc_timer_init_default2(
    &my_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    my_timer_callback,
    true);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_timer_init_default2.\n");
    return -1;
  } else {
    printf("Created timer with timeout %d ms.\n", timer_timeout);
  }

  // Assign message to publisher.
  example__msg__ExampleMessage__init(&pub_msg);
  const unsigned int PUB_MSG_CAPACITY = 20;
  pub_msg.message.data.data = allocator.reallocate(pub_msg.message.data.data, PUB_MSG_CAPACITY, allocator.state);
  pub_msg.message.data.capacity = PUB_MSG_CAPACITY;
  snprintf(pub_msg.message.data.data, pub_msg.message.data.capacity, "Hello World from C!");
  pub_msg.message.data.size = strlen(pub_msg.message.data.data);

  // Configure executor and spin.
  const unsigned int num_handles = 1;
  printf("Debug: number of DDS handles: %u\n", num_handles);
  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_spin(&executor);

  // Clean up (never called in this example)
  rc = rclc_executor_fini(&executor);
  rc += rcl_publisher_fini(&my_pub, &my_node);
  rc += rcl_timer_fini(&my_timer);
  rc += rcl_node_fini(&my_node);
  rc += rclc_support_fini(&support);
  example__msg__ExampleMessage__fini(&pub_msg);

  // Return final error code.
  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  return 0;
}