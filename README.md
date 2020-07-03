# youfork: Fully ROS 2 Homemade Mobile Manipulator

[![CircleCI](https://circleci.com/gh/youtalk/youfork.svg?style=svg)](https://circleci.com/gh/youtalk/youfork)

`youfork` is a mobile manipulator for home tidy-up. The movie below shows a little first try of the home tidy-up by teleoperation.

[![youfork: Fully ROS 2 Homemade Mobile Manipulator](https://img.youtube.com/vi/2srDav_n2S0/0.jpg)](https://www.youtube.com/watch?v=2srDav_n2S0)

[Slides from ROS Japan UG #37](https://docs.google.com/presentation/d/1QCLE6ED7YsSedzqXm65fToplTvbDD3roUvKMAm8AC5k/preview) (in Japanese)

All components are driven by ROS 2 Eloquent and Ubuntu 18.04 on the Jetson Xavier.
`youfork` has a lot of devices inside:

- NVIDIA Jetson Xavier
- ROBOTIS Open Manipulator X
- iRobot Roomba 643
- Intel RealSense D435i
- SlamTec RPLidar A1M8
- 25,000mAh Battery
- PS4 controller for teleoperation

`youfork` packages is beeing checked to build on CircleCI with ROS 2 Eloquent and Foxy.
However the Jetson Xavier on `youfork` is currently running only on Ubuntu 18.04, so that I've checked to work `youfork` properly only on ROS 2 Eloquent and Ubuntu 18.04.

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

## Launch nodes

### Bring up devices

```sh
. ~/youfork/install/setup.bash
ros2 launch ros2 launch youfork_bringup bringup.launch.xml use_create:=true use_open_manipulator:=true use_rplidar:=true use_realsense:=true
```

### Activate the Roomba lifecycle

```sh
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 lifecycle set /ca_driver configure
ros2 lifecycle set /ca_driver activate
```

### Visualize youfork by RViz2

```sh
. ~/youfork/install/setup.bash
rviz2 -d ~/youfork/src/youfork_description/config/youfork.rviz
```

### Teleoperate by PS4 controller

Firstly install [`ds4drv`](https://github.com/chrippa/ds4drv) and run the following command to connect with the wireless PS4 controller.

```sh
sudo ds4drv
```

Then launch the teleop related nodes.

```sh
. ~/youfork/install/setup.bash
ros2 launch youfork_teleop teleop.launch.xml
```
