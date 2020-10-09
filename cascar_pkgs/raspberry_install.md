# Instructions for Raspberry installation
Below are instructions for creating a ROS environment on the Raspberry Pi from scratch.

## Getting Ubuntu
First, download Ubuntu Mate disk image from (this was the distribution I think works best) from https://ubuntu-mate.org/raspberry-pi/ and put an an SD card. You’ll need a microSD card that is 6GB or greater. The file system will be automatically resized, on first boot, to occupy the unallocated space of the microSD card. Use a Class 10 microSDHC card or faster.

Boot from the card and follow the installation instructions. After boot, update all packages by
```bash
sudo apt-get update
sudo apt-get upgrade
```
Warning, this might take a while.

### Resize the boot partition (optional)
The boot partition is somewhat small, for example it is not possible to runt the software updater that comes with Ubuntu Mate. It is easy to resize the boot partition on a Linux laptio using, e.g., the ```gparted```application.


## Installing ROS
To install ROS, follow the instructions on http://wiki.ros.org/lunar/Installation/Ubuntu (for ROS Lunar).

It is also recommended to install the more modern build tools https://catkin-tools.readthedocs.io/en/latest/. Installation is done using

```bash
sudo apt-get update
sudo apt-get install python-catkin-tools
```

## Post installation instructions
Below are some post installation instructions.

### Enable ssh, camera, and serial port
Use command line tool ```rasp-config``` to activate ssh, camera, and serial port.

### Create wifi hotspot

### Switch between command line boot and graphical boot
To release memory, for example needed when compiling the qualisys ROS node, you might have to switch to a command line boot instead of a graphical boot to release memory. This can be done using the command line tool ```raspi-config```.


## Web browser
For the Ubuntu Mate, the latest version of Firefox crashes. One option is to downgrade version of Firefox or install another web browser, e.g., Chromium.

To install Chromium, do
```bash
sudo apt-get install chromium-browser
```

To downgrade Firefox, first determine available versions
```bash
apt-cache show firefox | grep Version
```
Then install a suitable version that works, for Ubuntu Mate 16.04 this works
```bash
sudo apt-get purge firefox
sudo apt-get install firefox=45.xxx
```
