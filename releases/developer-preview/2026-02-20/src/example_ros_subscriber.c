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
      printf("Published message %s\n", pub_msg.message.data);
    } else {
      printf("timer_callback: Error publishing message %s\n", pub_msg.message.data);
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
  rc = rclc_node_init_default(&my_node, "example_ros_subscriber", "example", &support);
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

  // create a timer, which will call the publisher with period=`timer_timeout` ms in the 'my_timer_callback'
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

  // create subscription
  rcl_subscription_t my_sub = rcl_get_zero_initialized_subscription();
  rc = rclc_subscription_init_default(
    &my_sub,
    &my_node,
    my_type_support,
    topic_name);
  if (rc != RCL_RET_OK) {
    printf("Failed to create subscriber %s.\n", topic_name);
    return -1;
  } else {
    printf("Created subscriber %s:\n", topic_name);
  }

  // one string message for subscriber
  example__msg__ExampleMessage__init(&sub_msg);

  ////////////////////////////////////////////////////////////////////////////
  // Configuration of RCL Executor
  ////////////////////////////////////////////////////////////////////////////
  rclc_executor_t executor;
  executor = rclc_executor_get_zero_initialized_executor();
  // total number of handles = #subscriptions + #timers
  //
  // Note:
  // If you need more than the default number of publisher/subscribers, etc., you
  // need to configure the micro-ROS middleware also!
  // See documentation in the executor.h at the function rclc_executor_init()
  // for more details.
  unsigned int num_handles = 1 + 1;
  printf("Debug: number of DDS handles: %u\n", num_handles);
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);

  // add subscription to executor
  rc = rclc_executor_add_subscription(
    &executor, &my_sub, &sub_msg, &my_subscriber_callback,
    ON_NEW_DATA);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_subscription. \n");
  }

  rclc_executor_add_timer(&executor, &my_timer);
  if (rc != RCL_RET_OK) {
    printf("Error in rclc_executor_add_timer.\n");
  }

  // Start Executor
  rclc_executor_spin(&executor);

  // clean up (never called in this example)
  rc = rclc_executor_fini(&executor);
  rc += rcl_subscription_fini(&my_sub, &my_node);
  rc += rcl_node_fini(&my_node);
  rc += rclc_support_fini(&support);

  example__msg__ExampleMessage__fini(&sub_msg);

  if (rc != RCL_RET_OK) {
    printf("Error while cleaning up!\n");
    return -1;
  }
  return 0;
}