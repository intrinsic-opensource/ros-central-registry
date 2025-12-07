# Copyright 2025 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Our container inherits from the smallest Ubuntu 24.04 footprint.
FROM ubuntu:24.04@sha256:278628f08d4979fb9af9ead44277dbc9c92c2465922310916ad0c46ec9999295

# Install baseline tools that are needed to build code and run tests.
RUN apt-get update                              \
 && apt-get install -y --no-install-recommends  \
        automake                                \
        autoconf                                \
        libtool                                 \
        python3                                 \
        sudo                                    \
        valgrind                                \
    && sudo rm -rf /var/lib/apt/lists/*

# Valgrind in docker must have a lower ulimit set. This makes sure this
# is changed before any test is run within the container.
# RUN echo "#!/bin/sh \n\
# set -e\n\
# sudo ulimit -n 65536 || true\n\
# exec \$@" > /entrypoint.sh
# RUN chmod 755 /entrypoint.sh
# ENTRYPOINT [ "/entrypoint.sh" ]
