#!/bin/bash

# Directory this file was executed from
echo "================================================================"
echo "Starting first time installation and setup, please wait"
echo "these downloads can take a while if you're on slow internet."
echo "================================================================"

# The current directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


echo "================================================================"
echo "Setting up your shell config files"
echo "================================================================"
# Shell config files that various shells source when they run.
# This is where we want to add aliases, source ROS environment
# variables, etc.
SHELL_CONFIG_FILES=(
    "$HOME/.bashrc"\
    "$HOME/.zshrc"
)

# All lines listed here will be added to the shell config files
# listed above, if they are not present already
declare -a new_shell_config_lines=(
    # Source the ROS Environment Variables Automatically
    "source /opt/ros/kinetic/setup.sh"\
    # Aliases to make development easier
    "alias clion=\"clion & disown && exit\""\
    "alias rviz=\"rviz & disown && exit\""\
    "alias rqt=\"rqt & disown && exit\""\
    "alias catkin=\"catkin \""\
    "alias run_tests=\"run_tests && catkin_test_results\""
)

# Add all of our new shell config options to all the shell
# config files, but only if they don't already have them
for file_name in "${SHELL_CONFIG_FILES[@]}"; 
do
    echo "Setting up $file_name"
    for line in "${new_shell_config_lines[@]}"; 
    do
        if ! grep -Fq "$line" $file_name 
        then
            echo "$line" >> $file_name 
        fi
    done
done


###############
# Install ROS #
###############

echo "================================================================"
echo "Installing ROS Kinetic"
echo "================================================================"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

#################
# Install CLion #
#################

echo "================================================================"
echo "Installing CLion"
echo "================================================================"
# Install dependencies
sudo apt-get install -y openjdk-8-jdk

# Fetch and extract CLion
echo "Fetching and extracting CLion"
wget https://download.jetbrains.com/cpp/CLion-2016.3.5.tar.gz
sudo tar xzf CLion*.tar.gz -C /usr/share
rm CLion*.tar.gz

# Run CLion Setup
cd /usr/share/clion*
./bin/clion.sh

# Make CLion globally accessible
echo "Linking CLion"
sudo ln -s -f /usr/share/clion*/bin/clion.sh /usr/local/bin/clion


##############################
# Install Other Dependencies #
##############################
cd $DIR
./install_dependencies.sh

echo "================================================================"
echo "Finished first time installation and setup; you're good to go!"
echo "================================================================"
