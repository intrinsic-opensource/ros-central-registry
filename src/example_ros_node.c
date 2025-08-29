#include <stdio.h>
#include <unistd.h> // For sleep()

#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rcl/node.h"
#include "rosidl_runtime_c/string_functions.h"
#include "std_msgs/msg/string.h"

// Helper function to check return codes and print errors
void check_ret(rcl_ret_t ret, const char * msg) {
    if (ret != RCL_RET_OK) {
        fprintf(stderr, "Error at %s: %s\n", msg, rcl_get_error_string().str);
        // Optional: exit or handle the error appropriately
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
    ret = rcl_node_init(&node, "minimal_c_node", "", &context, &node_options);
    check_ret(ret, "rcl_node_init");

    printf("Node created: %s\n", rcl_node_get_name(&node));

    // 3. Create a Publisher
    const char * topic_name = "chatter";
    const rosidl_message_type_support_t * type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();

    ret = rcl_publisher_init(&publisher, &node, type_support, topic_name, &publisher_options);
    check_ret(ret, "rcl_publisher_init");

    // 4. Create and publish messages in a loop
    std_msgs__msg__String msg;
    std_msgs__msg__String__init(&msg);
    const char * hello_world_str = "Hello, world! from C";
    // Allocate and copy string data
    rosidl_runtime_c__String__assign(&msg.data, hello_world_str);

    int count = 0;
    while (rcl_context_is_valid(&context)) {
        char buffer[100];
        snprintf(buffer, sizeof(buffer), "%s %d", hello_world_str, count++);
        rosidl_runtime_c__String__assign(&msg.data, buffer);

        ret = rcl_publish(&publisher, &msg, NULL);
        check_ret(ret, "rcl_publish");

        printf("Published: '%s'\n", msg.data.data);
        sleep(1); // Sleep for 1 second
    }

    // 5. Cleanup and Shutdown
    printf("Cleaning up...\n");
    std_msgs__msg__String__fini(&msg);
    ret = rcl_publisher_fini(&publisher, &node);
    check_ret(ret, "rcl_publisher_fini");
    ret = rcl_node_fini(&node);
    check_ret(ret, "rcl_node_fini");
    ret = rcl_shutdown(&context);
    check_ret(ret, "rcl_shutdown");
    ret = rcl_init_options_fini(&init_options);
    check_ret(ret, "rcl_init_options_fini");

    return 0;
}