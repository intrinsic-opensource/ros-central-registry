#include <stdlib.h>

#include "sensor_msgs/msg/detail/compressed_image__struct.h"
#include "sensor_msgs/msg/detail/compressed_image__functions.h"


int main(int arg, char* argv[]) {
    sensor_msgs__msg__CompressedImage *msg = sensor_msgs__msg__CompressedImage__create();
    sensor_msgs__msg__CompressedImage__destroy(msg);
    return 0;
}