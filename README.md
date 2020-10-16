# spinnaker_ros_interface

Spinnaker ROS Driver

[![Build Status](https://travis-ci.com/ut-amrl/spinnaker_ros_interface.svg?branch=master)](https://travis-ci.com/ut-amrl/spinnaker_ros_interface)


## Dependencies

1. [ROS](http://wiki.ros.org/Installation/)
1. The Spinnaker SDK, included under `spinnaker_driver`

## Build

1. Add the project directory to the `ROS_PACKAGE_PATH` environment variable.
    ```
    export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
    ```
1. Run `make`.

## Usage

1. Edit the default config file `config/blackfly-s.lua`, or create your own camera config file
1. Run `./bin/spinnaker_ros_interface` to run with the default config file
1. Run `./bin/spinnaker_ros_interface --config <file.lua>` to run with the specified config file
