# Raspberry Pi I2C Communication for ROS2

This package provides a simple I2C-Interface from Raspberry Pi to ROS2.
The following interfaces are tested

- rpi_i2c
  - DI
- rpi_piio
  - /

# Prerequisites

The installation assumes the Raspberry is running Ubuntu.
It has been tested with Ubuntu 20.04 LTS.

## Raspberry Setup
First, install `i2c-tools` to run commands from the terminal

    sudo apt install i2c-tools

Add `ubuntu` (or the user running the programs) to the `i2c` group

    sudo usermod -a -G i2c ubuntu

List all i2c devices

    i2cdetect -l

List all i2d slaves

    sudo i2cdetect -y 1

## Enable access to i2c

Next we have to edit the modules file and add a new line i2c-dev at the end of the module file.
Open `/etc/modules` in an editor.

    sudo nano /etc/modules

After then it appears something like this below in your cmd shell.

    # /etc/modules: kernel modules to load at boot time.
    #
    # This file contains the names of kernel modules that should be loaded
    # at boot time, one per line. Lines beginning with "#" are ignored.
    # Parameters can be specified after the module name.

Add the following two lines to the file

    snd-bcm2835
    i2c-dev

Close, save, and reboot.

## Python

The smbus library provides basic functionality for I2C handling in python. Install using `pip3`

    pip3 install cffi
    pip3 install smbus-cffi

Also, the bitarray library is implicitly used. Install using 'pip3'

    pip3 install bitstring

## PiIO

You need to run several commands in the upper PiIO directory:

  ./install_packages.sh
  ./install_py_packages.sh
  sudo python3 ./setup.py install

# Usage on Device

## Compiling

Since the system was probably initialized before, the system will put out errors when trying to build with colcon (you should also add --packages-ignore module_comm , this package is not compilable). The workspace was built before with root permissions and it somehow messes things up when rebuilding without sudo permissions. To fix that run following command in the workspace root folder:

    sudo rm -i -f build/ log/ install/

## Setup

After that run:

    sudo -i

anywhere and redirect to your workspace root folder.

## Build

The following command build all packages in this repo.
It should be run in the ros2 workspace.

    colcon build --packages-select rpi_i2c_msgs rpi_i2c rpi_piio

## Run

Then you can source:

    source install/setup.bash

and run

    ros2 launch rpi_piio test.launch.py
