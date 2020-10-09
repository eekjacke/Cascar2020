# REWORKING

# Installation instructions for cascar catkin workspace
Installation instructions for installing required packages for cascar operation. The installation requires a full ROS installation already installed, this has been tested on Noetic ROS distribution. The build system will rely on the new catkin build tools https://catkin-tools.readthedocs.io/

Installation instructions for the packages below are included. Note that on the Raspberry Pi, only cascar and RPLidar nodes are typically installed due to computational and memory constraints. Note also the udev-rules installation instructions that should be done on both the car and the control laptop.

* cascar - our RC car
* Qualisys - positioning system
* RPLidar - RPLidar ROS drivers
* Joystick drivers

## Required Ubuntu packages
TBD

## Create and initialize workspace
See https://catkin-tools.readthedocs.io/ for detailed instructions. To create and initialize workspace, do
```bash
mkdir -p cascar_ws/src
cd cascar_ws
catkin init
```

## cascar
Extract or clone git repository in the ```src``` directory
```bash
cd cascar_ws/src
git clone git@gitlab.ida.liu.se:fs/cascar_pkgs.git
```
and then build the workspace
```bash
catkin_make
```
and finally source the ```setup.bash```
```bash
cd catkin_ws
source devel/setup.bash
```

## Joystick
Installation of the ROS package http://wiki.ros.org/joystick_drivers

```bash
cd catkin_ws/src
git clone https://github.com/ros-drivers/joystick_drivers.git
catkin build
source ../devel/setup.bash
```

## Cartographer
Installation of the ROS package https://github.com/googlecartographer/cartographer_ros follows the installation instructions, but substituting the build system. This build takes a lot of time.

```bash
cd cascar_ws
wstool init src

# Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

# Install proto3.
src/cartographer/scripts/install_proto3.sh

# Install deb dependencies.
# The command 'sudo rosdep init' will print an error if you have already
# executed it since installing ROS. This error can be ignored.
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build and install.
catkin build # This differs from package installation instructions
source ../devel/setup.#!/usr/bin/env bash
```

## Udev rules
Device rules installation instructions, ensuring that the car and joystick are mapped to the same device each time. These instructions in based on https://www.sparkfun.com/news/2332.

For cascar/RCCar, put file ```src/cascar/develop/udevrules/cascar.rules``` in directory ```/etc/udev/rules.d```. For the Logitech Extreme 3D joystick, put
file ```src/cascar/develop/udevrules/logitech_extreme_3d.rules``` in directory ```/etc/udev/rules.d```.

If you have other devices; for device information and how to activate device rules, use
1. dmesg for device mount information
2. udevadm info -a -p  $(udevadm info -q path -n /dev/ttyUSB0) for details
3. udevadm trigger -- to activate rules without reboot, for debuggning
