# Installs all the required ros packages
# (should really be depreceated in favour of using rosdeps properly)

sudo apt-get install -y \
    libeigen3-dev \
    ros-kinetic-urg-node
#!/bin/bash

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Installs all required dependencies to build this repo
rosdep install --from-paths $CURR_DIR --ignore-src --rosdistro=ROSDISTRO
