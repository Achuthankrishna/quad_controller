## PX-4 off-board Velocity Control and Teleop

This package consists of C++ implementation of teleoperation and velocity controller for PX4 in ROS2. This project was inspired by python implementation done by : https://github.com/Jaeyoung-Lim/px4-offboard

(./img/drone_Vid_warehouse-ezgif.com-video-to-gif-converter.gif)

## Prerequisites
- ROS2 Humble
- PX4 Autopilot
- Micro XRCE-DDS Agent
- px4_msgs
- OpenCV (Further implementation of vison based obstacle avoidance)

## Installations
Some of the steps to install the dependency packages : 

### Install PX4 Autopilot
To [Install PX4](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets) run this code 
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

Run this script in a bash shell to install everything

```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

**Restart once done**

### Install Dependencies

Install Python dependencies as mentioned in the [PX4 Docs](https://docs.px4.io/main/en/ros/ros2_comm.html#install-ros-2) with this code

```
pip3 install kconfiglib
pip install --user jsonschema
pip install --user jinja2
```

### Build Micro DDS
Micro DDS is used for communication between fastRTPS(ROS2) and u-ORB.
```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
## Workspace Setup

- Open your ROS2_wS
```
cd ~/ros2_px2/src
```

- Clone the PX4_msgs repo inside src first. This helps in ROS messages for PX4 commands.Be sure you're in the src directory of your workspace and then run this code to clone in the px4_msgs repo

```
git clone https://github.com/PX4/px4_msgs.git
```
- Clone the package inside SRC

```
git clone https://github.com/Achuthankrishna/quad_controller.git
```

Run this code to clone the repo



### Building the Workspace

- Before building these two packages, source your ROS2 installation . Run this code to do that

```
source /opt/ros/humble/setup.bash
```
- Now build the packages

```
cd ..
colcon build
```

- Source the workspace again
```
source install/setup.bash
```

## Running the Code

- Split your terminal into 3 parts. IN the forst part get into the PX4-autopilot folder and then type the following
```
make px4_sitl gazebo-classic_iris_depth_camera__warehouse

```
- Then in the second terminal, lets start the udp4 agent for micro-ros connection to connect to port 8888. For in-depth theory, check out [PX4 Documantation](https://docs.px4.io/main/en/middleware/uxrce_dds.html)

```
micro-ros-agent udp4 --port 8888

```
- In the Third terminal,Open our ros2 workspace, sourced and type the following 
```
ros2 launch quad_off_board teleop_launch.py
```


