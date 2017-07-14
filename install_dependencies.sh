#!/bin/bash

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Installs all required dependencies to build this repo
rosdep install --from-paths $CURR_DIR --ignore-src --rosdistro=ROSDISTRO
