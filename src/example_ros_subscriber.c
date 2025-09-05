#include <stdio.h>

#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rcl/node.h"
#include "rcl/subscription.h"
#include "rcl/wait.h"
#include "std_msgs/msg/string.h"
#include "rosidl_runtime_c/string_functions.h"

// Helper function to check return codes and print errors
void check_ret(rcl_ret_t ret, const char * msg) {
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Error at %s: %s\n", msg, rcl_get_error_string().str);
        // In a real application, you might want more robust error handling
    }
}

int main(int argc, const char * const* argv) {
    rcl_ret_t ret;

    // 1. Initialize RCL context and allocator
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
    check_ret(ret, "rcl_init_options_init");

    rcl_context_t context = rcl_get_zero_initialized_context();
    ret = rcl_init(argc, argv, &init_options, &context);
    check_ret(ret, "rcl_init");

    // 2. Create the Node
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(&node, "minimal_c_subscriber", "", &context, &node_options);
    check_ret(ret, "rcl_node_init");

    printf("Node created: %s\n", rcl_node_get_name(&node));

    // 3. Create a Subscriber
    const char * topic_name = "chatter";
    const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    
    rcl_subscription_t subscriber = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();

    ret = rcl_subscription_init(&subscriber, &node, type_support, topic_name, &subscription_options);
    check_ret(ret, "rcl_subscription_init");
    printf("Subscriber created for topic '%s'\n", topic_name);


    // 4. Create a Wait Set to wait for messages
    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    // Initialize with 1 subscription, and 0 of everything else.
    ret = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context, rcl_get_default_allocator());
    check_ret(ret, "rcl_wait_set_init");

    // 5. Main loop to wait for and process messages
    printf("Listening for messages...\n");
    while (rcl_context_is_valid(&context)) {
        // Clear the wait set
        ret = rcl_wait_set_clear(&wait_set);
        check_ret(ret, "rcl_wait_set_clear");

        // Add the subscription to the wait set
        size_t index;
        ret = rcl_wait_set_add_subscription(&wait_set, &subscriber, &index);
        check_ret(ret, "rcl_wait_set_add_subscription");

        // Wait for a new message to arrive (blocking call)
        // A timeout of 1 second is used here.
        ret = rcl_wait(&wait_set, RCL_MS_TO_NS(1000));
        
        if (ret == RCL_RET_TIMEOUT) {
            // If timeout, just continue the loop
            continue;
        }
        check_ret(ret, "rcl_wait");

        // Check if our subscription has received a message
        if (wait_set.subscriptions[0]) {
            std_msgs__msg__String msg;
            std_msgs__msg__String__init(&msg);
            
            rmw_message_info_t message_info; // Not used, but required by rcl_take
            
            ret = rcl_take(&subscriber, &msg, &message_info, NULL);

            if (ret == RCL_RET_OK) {
                 printf("I heard: '%s'\n", msg.data.data);
            } else if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
                // This error means no message was available, which can happen.
                // Other errors are more serious and should be checked.
                check_ret(ret, "rcl_take");
            }

            // Clean up the message memory
            std_msgs__msg__String__fini(&msg);
        }
    }

    // 6. Cleanup and Shutdown
    printf("Cleaning up...\n");
    ret = rcl_subscription_fini(&subscriber, &node);
    check_ret(ret, "rcl_subscription_fini");
    ret = rcl_wait_set_fini(&wait_set);
    check_ret(ret, "rcl_wait_set_fini");
    ret = rcl_node_fini(&node);
    check_ret(ret, "rcl_node_fini");
    ret = rcl_shutdown(&context);
    check_ret(ret, "rcl_shutdown");
    ret = rcl_init_options_fini(&init_options);
    check_ret(ret, "rcl_init_options_fini");

    return 0;
}
