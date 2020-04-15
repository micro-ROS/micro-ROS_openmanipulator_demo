# micro-ROS OpenManipulator-X Demo

TODO: Introduction text

![](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/micro-ROS/micro-ROS_openmanipulator_demo/master/assets/diagrams/entities_diagram.puml)

TODO: Introduction text


![](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.githubusercontent.com/micro-ROS/micro-ROS_openmanipulator_demo/master/assets/diagrams/usecase_diagram.puml)

## Hardware

The following is a list of the hardware needed to reproduce this demo:

* 1 x [Robotis OpenMANIPULATOR-X](http://www.robotis.us/openmanipulator-x/)
* 1 x [Robotis OpenMANIPULATOR-X Power Supply](http://www.robotis.us/smps-12v-5a-ps-10-us-110v/)
* 1 x [Robotis U2D2](http://www.robotis.us/u2d2/)
* 1 x [Pimoroni VL53LX ToF sensor](https://shop.pimoroni.com/products/vl53l1x-breakout)
* 1 x [Raspberry Pi 4](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/) + Power Supply
* 1 x [Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)
* 1 x USB to microUSB cable
* 1 x USB to miniUSB cable


## How to build

Instructions:

Install in Ubuntu RPi Imager
Download and burn Ubuntu
On the SD you can set your wifi password
SSH enabled by default: ubuntu:ubuntu

-- Create SWAP:

```bash
sudo fallocate -l 1G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo nano /etc/fstab 
        /swapfile swap swap defaults 0 0
sudo free -h
```

-- Install Dashing:

```bash
sudo apt update && sudo apt upgrade
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update && sudo apt install ros-dashing-ros-base
```

-- Install OpenManipulator packages:

```bash
sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev
mkdir -p ~/robotis_ws/src && cd ~/robotis_ws/src
git clone -b ros2 https://github.com/ROBOTIS-GIT/DynamixelSDK.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/dynamixel-workbench.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/robotis_manipulator.git 
```


```
cd ~/robotis_ws/
echo 'source /opt/ros/dashing/setup.bash' >> ~/.bashrc
source ~/.bashrc
colcon build --symlink-install --parallel-workers 1
```


UDEV RULES
```
sudo cp /home/ubuntu/robotis_ws/src/open_manipulator/open_manipulator_x_controller/99-open-manipulator-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## How to use

ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py  
ros2 launch open_manipulator_x_description open_manipulator_x_rviz2.launch.py 


## How to run RViz


## Purpose of the project

The software is not ready for a production use.
It has neither been developed nor tested for a specific use case.
However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs.
Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust ot according to any applicable safety standards (e.g. ISO 26262).