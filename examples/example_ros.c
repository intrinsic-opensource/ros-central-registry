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

#include <stdlib.h>

#include "sensor_msgs/msg/detail/compressed_image__struct.h"
#include "sensor_msgs/msg/detail/compressed_image__functions.h"


int main(int arg, char* argv[]) {
    sensor_msgs__msg__CompressedImage *msg = sensor_msgs__msg__CompressedImage__create();
    sensor_msgs__msg__CompressedImage__destroy(msg);
    return 0;
}