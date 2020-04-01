open_manipulator_x_tof





Instructions:

Install in Ubuntu RPi Imager
Download and burn Ubuntu
On the SD you can set your wifi password
SSH enabled by default: ubuntu:ubuntu

-- Create SWAP:

sudo fallocate -l 1G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo nano /etc/fstab
        /swapfile swap swap defaults 0 0
sudo free -h

-- Install Dashing:

sudo apt update && sudo apt upgrade
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update && sudo apt install ros-dashing-ros-base


-- Install OpenManipulator packages:

sudo apt install python3-argcomplete python3-colcon-common-extensions libboost-system-dev
mkdir -p ~/robotis_ws/src && cd ~/robotis_ws/src
git clone -b ros2 https://github.com/ROBOTIS-GIT/DynamixelSDK.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/dynamixel-workbench.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git  
git clone -b ros2 https://github.com/ROBOTIS-GIT/robotis_manipulator.git 

CHECK THIS
cd ~/robotis_ws/src/turtlebot3
rm -r turtlebot3_cartographer turtlebot3_navigation2

cd ~/robotis_ws/
echo 'source /opt/ros/dashing/setup.bash' >> ~/.bashrc
source ~/.bashrc
colcon build --symlink-install --parallel-workers 1


UDEV RULES
sudo cp /home/ubuntu/robotis_ws/src/open_manipulator/open_manipulator_x_controller/99-open-manipulator-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger