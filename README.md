# spinnaker_ros_interface

Spinnaker ROS Driver

[![Build Status](https://travis-ci.com/ut-amrl/spinnaker_ros_interface.svg?branch=master)](https://travis-ci.com/ut-amrl/spinnaker_ros_interface)


## Dependencies

1. [ROS](http://wiki.ros.org/Installation/)
1. The Spinnaker SDK found at [FLIR](https://meta.box.lenovo.com/v/link/view/a1995795ffba47dbbe45771477319cc3).
    - Navigate to the download directory, unzip the folder, navigate into the folder and run:
    ```
    ./install_spinnaker.sh
    ```
1. The AMRL shared library [amrl_shared_lib](https://github.com/ut-amrl/amrl_shared_lib) and AMRL configuration reader [config_reader](https://github.com/ut-amrl/config-reader). NOTE that both are installed during submodule initialization in the Build instructions by default.
1. Install dependencies:
    ```
    sudo apt install liblua5.1-0-dev
    ```

## Build

1. Navigate to the `spinnaker_ros_interface` directory.

1. Initialize the submodules:
    ```
    git submodule update --init
    ```

1. Add the project directory to the `ROS_PACKAGE_PATH` environment variable:
    ```
    export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
    ```


1. Run `make`

## Usage

1. Edit the default config file `config/blackfly-s.lua`, or create your own camera config file
1. Run `./bin/spinnaker_ros_interface` to run with the default config file
1. Run `./bin/spinnaker_ros_interface --config <file.lua>` to run with the specified config file
