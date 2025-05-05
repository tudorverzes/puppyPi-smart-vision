# PuppyPi Smart Vision

## Project Description

This project integrates real-time image processing with the control of a quadruped robot, PuppyPi. The purpose of the application is to detect a red object and command the robot to move towards it, keeping it centered in the visual field.

The application was initially developed for a robotics lab but is also highly relevant to the IoT field, as it involves:

- Sensors (video camera)
- Microcontrollers (Raspberry Pi)
- Network communication
- Local automation and remote control capabilities

## Robot Architecture

PuppyPi is a quadruped robot equipped with:

- Raspberry Pi 4B (ARM Cortex-A72 processor, 4GB RAM)
- 16-DOF PWM servo driver board
- HD Camera
- Battery
- Operating system based on ROS (Robot Operating System)

## Pre-requisites

### Hardware

- Hiwonder PuppyPi robot (pre-assembled) with included video camera
- Raspberry Pi 4B (with a minimum 16GB SD card)
- Rechargeable battery included
- Network access via Wi-Fi

### Software

- Ubuntu 20.04 (ARM) for Raspberry Pi
- ROS Noetic
- Python 3.11
- OpenCV
- ROS packages: sensor_msgs, std_srvs, puppy_control, cv_bridge

## Setup and Build

1. Connect to the robot's Wi-Fi Direct network.
2. Access the robot's graphical interface using RealVNC Viewer.
3. Once connected, open the bash console and enter the following commands to set up the system and dependencies:

    1. Set up the system
        ```bash
        sudo apt update
        sudo apt install ros-noetic-desktop-full python3-catkin-tools python3-opencv
        ```
    
    2. Create the ROS Workspace
        ```bash
        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws/src
        catkin_init_workspace
        cd ..
        catkin build
        source devel/setup.bash
        ```

    3. Install the PuppyPi package
        ```bash
        cd ~/catkin_ws/src
        git clone https://github.com/hiwonder/puppy_pi_ros.git
        cd ..
        catkin build
        ```

    4. Add our custom script
        ```bash
        cd ~/catkin_ws/src
        git clone https://github.com/tudorverzes/puppyPi-smart-vision.git
        chmod +x ~/catkin_ws/src/puppyPi-smart-vision/src/pkg_1/scripts/move_image_processing_node.py
        ```

## Running the Program

Run the program from the console:

```bash
cd ~/catkin_ws/src/puppyPi-smart-vision/src/pkg_1/scripts
./move_image_processing_node.py
