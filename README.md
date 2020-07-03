# youfork: Fully ROS 2 Homemade Mobile Manipulator

[![youfork: Fully ROS 2 Homemade Mobile Manipulator](https://img.youtube.com/vi/2srDav_n2S0/0.jpg)](https://www.youtube.com/watch?v=2srDav_n2S0)

 youfork packages is beeing checked to build on CircleCI with ROS 2 Eloquent and Foxy.

[![CircleCI](https://circleci.com/gh/youtalk/youfork.svg?style=svg)](https://circleci.com/gh/youtalk/youfork)

However the Jetson Xavier on youfork is currently running only on Ubuntu 18.04, so that I've checked to work youfork properly only on ROS 2 Eloquent and Ubuntu 18.04.

## Install dependencies

Firstly install `librealsense2` package because it cannot be installed by `rosdep` then check out repositories by `vcs import` and run `rosdep install`.

```sh
cd ~/
git clone git@github.com:youtalk/youfork.git
cd ~/youtalk
sudo apt update
sudo apt install -y software-properties-common
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt update
sudo apt install --no-install-recommends python3-vcstool librealsense2-dev
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/youtalk
vcs import src < youfork.repos
rosdep install --from-paths . --ignore-src -y
```

## Build

```sh
cd ~/youtalk
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
```
