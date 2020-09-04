#!/usr/bin/env bash

# Unfreeze update by updating ROS's keys
# remove the old key
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
# add the new key
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update -y && sudo apt-get upgrade -y

sudo apt-get install -y \
    python-pyaudio \
    luakit \
    ros-${ROS_DISTRO}-audio-common \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    dphys-swapfile
