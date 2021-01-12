# spinnaker_ros_interface

Spinnaker ROS Driver

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
1. In the terminal source your ROS install 
    ```
    source /opt/ros/noetic/setup.bash 
    ```
1. Navigate to the `spinnaker_ros_interface` directory.

1. Initialize the submodules:
    ```
    git submodule update --init
    ```
    
1. Run `make`

## Usage
You can run the node manually from the command line or use roslaunch.
### Commnad Line
Before running the node you must start a `roscore` in another terminal. 
1. Edit the default config file `config/blackfly-s.lua`, or create your own camera config file
1. Run `./bin/spinnaker_ros_interface` to run with the defaults
1. Run `./bin/spinnaker_ros_interface --config <file.lua>` to run with the specified config file
1. Run  `./bin/spinnaker_ros_interface __name:=<node_name>` to run with user specified ROS node name - required when running two or more nodes

### Roslaunch
From the `spinnaker_ros_interface` directory add the package to the `ROS_PACKAGE_PATH` so roslaunch will be able to find it:
1. ``` export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd` ```
1. Run `roslaunch spinnaker_ros_interface spinnaker_ros_interface.launch` to run with the defaults
1. Run `roslaunch spinnaker_ros_interface spinnaker_ros_interface.launch camera_config:=<config_file.lua>` to run with the specified config file
1. Run `roslaunch spinnaker_ros_interface spinnaker_ros_interface.launch node_name:=<unique_node_name>` to run with user specified ROS node name - required when running two or more nodes

Note that users who are familiar with launch files can edit these parameters in the launch file itself and take advantage of the control/flexibility that [roslaunch](https://wiki.ros.org/roslaunch) gives users.

## Spinnaker SpinView QT Application

The SDK provides a GUI with which you can verify camera connectivity and find information like camera serial number.

1. Navigate to the Spinnaker binary directory 
    ```
    cd /opt/spinnaker/bin
    ```
1. Run SpinView application
    ```
    ./SpinView_QT
    ```
In order to configure synchronized capture see [this link](https://www.flir.com/support-center/iis/machine-vision/application-note/configuring-synchronized-capture-with-multiple-cameras/)
