# ros-humble-ros1-bridge-builder
Create a "*ros-humble-ros1-bridge*" package that can be used directly within Ubuntu 22.02 (Jammy) ROS2 Humble. Both amd64 and arm64 architectures are supported.

- Note1: It takes approximately 10 minutes on my PC, equipped with a 6-core CPU (12 logical cores) and 24GB of memory.

- Note2: It takes about 1 GB of memory per logical CPU core to compile the ROS1 bridge. So, if your system has only 4 GB of memory but 100 logical CPU cores, it will still use only 4 logical cores for the compilation. Now, why does it take so much memory to compile?  Well, you can blame the overuse of C++ templates...

- Note3: If you are looking for ROS2 Jazzy + Ubuntu 24.04 support, see https://github.com/TommyChangUMD/ros-jazzy-ros1-bridge-builder

## System Architecture and Bridge Purpose
This Docker-based setup allows seamless communication between a robot's existing ROS 1 Noetic system (running natively on a base PC) and new software components developed in ROS 2 Humble.

To avoid modifying or risking the stability of the host system (Ego base PC running Ubuntu 20.04 with ROS 1 Noetic), the ROS 1–ROS 2 bridge is deployed inside a Docker container.

The full architecture on the base PC consists of:

- ROS1 Noetic – Running directly on the host (Ego base PC).

- Container A – Runs ROS2 Humble (used for developing or testing new software nodes).

- Container B – Runs both ROS1 and ROS2 along with the ros1_bridge to enable bidirectional communication between the two systems.

This configuration ensures isolation between environments while enabling cross-communication through the bridge.

## How to create this builder docker images:

``` bash
  mkdir Docker_folders
  cd Docker_folders
  git clone https://github.com/IIT-SoftBots/ros-humble-ros1-bridge-builder.git
  cd ros-humble-ros1-bridge-builder

  # By default, ros-tutorals support will be built: (bridging the ros-humble-example-interfaces package)
  docker build . -t ros-humble-ros1-bridge-builder --network host
```

<!-- - Note1: Since building a docker image just needs docker, you could do this step on any system that has docker installed -- it doesn't have to on a Ubuntu 22.04 (Jammy) and it doesn't need ROS2 neither.

- Note2: The builder image can be created on an amd64 machine (e.g., Intel and AMD CPUs) or an arm64 machine (e.g., Raspberry Pi 4B and Nvidia Jetson Orin).  Docker will automatically select the correct platform variant based on the host's architecture.


Alternative builds:
``` bash
  # **[OPTIONAL]** If you don't want to build ros-tutorals support:
  docker build . --build-arg ADD_ros_tutorials=0 -t ros-humble-ros1-bridge-builder

  # **[OPTIONAL]** If you want to build grid-map support:  (bridging the ros-humble-grid-map package)
  docker build . --build-arg ADD_grid_map=1 -t ros-humble-ros1-bridge-builder

  # **[OPTIONAL]** If you want to build an example custom message:
  docker build . --build-arg ADD_example_custom_msgs=1 -t ros-humble-ros1-bridge-builder

  # **[OPTIONAL]** If you want to build octomap:
  docker build . --build-arg ADD_octomap_msgs=1 -t ros-humble-ros1-bridge-builder

  # **[OPTIONAL]** If you want to build octomap and grid-map together:
  docker build . --build-arg ADD_octomap_msgs=1 --build-arg ADD_grid_map=1 -t ros-humble-ros1-bridge-builder

```
- Note1: Don't forget to install the necessary `ros-humble-grid-map` packages on your ROS2 Humble if you choose to build the bridge with the `grid-map` support added.

- Note2: For the custom message example, there is no pre-build package for ROS2 Humble so you will need to compile it from the source.  For details, see [Checking example custom message](#checking-example-custom-message) in the Troubleshoot section. -->

## How to create ros-humble-ros1-bridge package:
###  0.) Running a Temporary Docker Container with Host Networking

``` bash
    cd ~/
    docker run -it --rm --network host ros-humble-ros1-bridge-builder bash
```

## How to use ros-humble-ros1-bridge:
###  1.) First start ROS1 Noetic node on the host PC that could be Alter-Ego PC or you personal PC with ROS1 Noetic or ROS1 Noetic Docker:
### simple test:

``` bash
rosrun rospy_tutorials listener
```

###  2.) Then, starts ros1_bridge node inside the ros-humble-ros1-bridge docker container

``` bash
  source /opt/ros/humble/setup.bash
  source /ros-humble-ros1-bridge/install/local_setup.bash
  ros2 run ros1_bridge dynamic_bridge
  # or try (See Note2):
  ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

###  3.) Now, starts ROS2 Humble system, inside the ROS2 Humble docker container 
https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html

``` bash
docker pull osrf/ros:humble-desktop
docker run -it osrf/ros:humble-desktop
ros2 run demo_nodes_cpp talker
```
###  4.) Finally, check if the communication is working, check rostopic list from both side ROS1 and ROS2:

``` bash
  source /opt/ros/humble/setup.bash
  ros2 run demo_nodes_cpp listener
```

## How to use parameter_bridge instead of dynamic_bridge

In the config folder, there is a file named parameter_bridge_template.yaml, which depends on the ROBOT_NAME variable — a common environment variable typically defined in the robot’s base computer .bashrc file.

``` bash
  docker build . -t ros-humble-ros1-bridge-builder --network host
  docker run -it --rm --network host --name ${ROBOT_NAME}_bridge --mount src=~/Docker_folders/ros-humble-ros1-bridge-builder/config/parameter_bridge_template.yaml,target=/parameter_bridge_template.yaml,type=bind --mount src=~/.bashrc,target=/host_bashrc,type=bind -e ROBOT_NAME=$ROBOT_NAME ros-humble-ros1-bridge-builder:latest bash
```
Once inside the Docker container, run:
``` bash
parameter_bridge
```
This command runs parameter_bridge.sh (which is located in the bin folder of the container’s repository). This script generates the parameter_bridge.yaml file based on the parameter_bridge_template.yaml file. As you can see, the resulting parameter_bridge.yaml inside the container is now configured according to the robot’s name (ROBOT_NAME).

## How to add custom message from ROS1 and ROS2 source code to use it with parameter_bridge
If you want to include topics with custom messages in the parameter_bridge_template.yaml file, you need to enable it in the Dockerfile by setting the variable ARG ADD_alterego_custom_msgs=1.
After that, customize the parameter_bridge_template.yaml file again and then run:

``` bash
  docker build . -t ros-humble-ros1-bridge-builder --network host
  docker run -it --rm --network host --name ${ROBOT_NAME}_bridge --mount src=~/Docker_folders/ros-humble-ros1-bridge-builder/config/parameter_bridge_template.yaml,target=/parameter_bridge_template.yaml,type=bind --mount src=~/.bashrc,target=/host_bashrc,type=bind -e ROBOT_NAME=$ROBOT_NAME ros-humble-ros1-bridge-builder:latest bash

```
Once inside the Docker container, run:
``` bash
parameter_bridge
```
## How to check if ros-humble-ros1-bridge is working with parameter_bridge:
Remember to start ROS1 on the host PC. In this case, to test the bridge, launch the Alter-Ego simulator by running the following file, as usual:
``` bash
roslaunch alterego_gazebo main.launch AlterEgoVersion:=3
```
Now, open a new terminal in the Docker container and check if the bridge can see the topics:
``` bash
docker exec -it container_name bash
source /opt/ros/humble/setup.bash
source /root/ros1_ws/src/alterego_msgs/install/setup.bash
source /root/ros2_ws/src/alterego_msgs/install/setup.bash
```
Choose one of the topics you customized in the parameter_bridge_template.yaml file and check if values are being received.
``` bash
ros2 topic echo /ROBOT_NAME/alterego_state/lowerbody | grep mobile_base_pos_x
```
<!-- ## How to add custom message from ROS1 and ROS2 source code
See an step 6.3 and 7 in the Dockerfile for an example.

- Note1: Make sure the package name ends with "_msgs".
- Note2: Use the same package name for both ROS1 and ROS2.

Also see the [troubleshoot section](#checking-example-custom-message).

- ref: https://github.com/TommyChangUMD/custom_msgs.git
- ref: https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst


## How to make it work with ROS1 master running on a different machine?
- Run `roscore` on the Noetic machine as usual.
- On the Humble machine, run the bridge as below (assuming the IP address of the Noetic machine is 192.168.1.208):

``` bash
  source /opt/ros/humble/setup.bash
  source ~/ros-humble-ros1-bridge/install/local_setup.bash
  ROS_MASTER_URI='http://192.168.1.208:11311' ros2 run ros1_bridge dynamic_bridge
  # Note, change "192.168.1.208" above to the IP address of your Noetic machine.
```

## Troubleshoot

### Fixing "[ERROR] Failed to contact master":

If you have Noetic and Humble running on two different machines and have
already set the ROS_MASTER_URI environment variable, you should check the
network to ensure that the Humble machine can reach the Noetic machine via
port 11311.

``` bash
$ nc -v -z 192.168.1.208 11311
# Connection to 192.168.1.208 11311 port [tcp/*] succeeded!
```

### Checking tf2 message / service:
``` bash
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i tf2
  - 'tf2_msgs/msg/TF2Error' (ROS 2) <=> 'tf2_msgs/TF2Error' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf2_msgs/TFMessage' (ROS 1)
  - 'tf2_msgs/msg/TFMessage' (ROS 2) <=> 'tf/tfMessage' (ROS 1)
  - 'tf2_msgs/srv/FrameGraph' (ROS 2) <=> 'tf2_msgs/FrameGraph' (ROS 1)
```

### Checking AddTwoInts message / service:
- By default, `--build-arg ADD_ros_tutorials=1` is implicitly added to the `docker build ...` command.
- The ROS2 Humble system must have the `ros-humble-example-interfaces` package installed.
``` bash
$ sudo apt -y install ros-humble-example-interfaces
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i addtwoints
  - 'example_interfaces/srv/AddTwoInts' (ROS 2) <=> 'roscpp_tutorials/TwoInts' (ROS 1)
  - 'example_interfaces/srv/AddTwoInts' (ROS 2) <=> 'rospy_tutorials/AddTwoInts' (ROS 1)
```

### Checking grid-map message / service:
- Must have `--build-arg ADD_grid_map=1` added to the `docker build ...` command.
- Note: In addition, the ROS2 Humble system must have the `ros-humble-grid-map` package installed.
``` bash
$ sudo apt -y install ros-humble-grid-map
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i grid_map
  - 'grid_map_msgs/msg/GridMap' (ROS 2) <=> 'grid_map_msgs/GridMap' (ROS 1)
  - 'grid_map_msgs/msg/GridMapInfo' (ROS 2) <=> 'grid_map_msgs/GridMapInfo' (ROS 1)
  - 'grid_map_msgs/srv/GetGridMap' (ROS 2) <=> 'grid_map_msgs/GetGridMap' (ROS 1)
  - 'grid_map_msgs/srv/GetGridMapInfo' (ROS 2) <=> 'grid_map_msgs/GetGridMapInfo' (ROS 1)
  - 'grid_map_msgs/srv/ProcessFile' (ROS 2) <=> 'grid_map_msgs/ProcessFile' (ROS 1)
  - 'grid_map_msgs/srv/SetGridMap' (ROS 2) <=> 'grid_map_msgs/SetGridMap' (ROS 1)
```

### Checking example custom message:
- Thanks to [Codaero](https://github.com/Codaero) for the source code for an custom message example.
- Must have `--build-arg ADD_example_custom_msgs=1` added to the `docker build ...` command.
``` bash
# First, install the ROS2 pacakge from the source
$ git clone https://github.com/TommyChangUMD/custom_msgs.git
$ cd custom_msgs/custom_msgs_ros2
$ source /opt/ros/humble/setup.bash
$ colcon build
$ source install/setup.bash

# Now, run the bridge
$ source ~/ros-humble-ros1-bridge/install/local_setup.bash
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i PseudoGridMap
  - 'custom_msgs/msg/PseudoGridMap' (ROS 2) <=> 'custom_msgs/PseudoGridMap' (ROS 1)
```

### Checking octomap message:
- Must have `--build-arg ADD_octomap_msgs=1` added to the `docker build ...` command.
- Note: In addition, the ROS2 Humble system must have the `ros-humble-octomap-msgs` package installed.``` bash
$ sudo apt -y install ros-humble-octomap-msgs
$ ros2 run ros1_bridge dynamic_bridge --print-pairs | grep -i octomap
  - 'octomap_msgs/msg/Octomap' (ROS 2) <=> 'octomap_msgs/Octomap' (ROS 1)
  - 'octomap_msgs/msg/OctomapWithPose' (ROS 2) <=> 'octomap_msgs/OctomapWithPose' (ROS 1)
  - 'octomap_msgs/srv/BoundingBoxQuery' (ROS 2) <=> 'octomap_msgs/BoundingBoxQuery' (ROS 1)
  - 'octomap_msgs/srv/GetOctomap' (ROS 2) <=> 'octomap_msgs/GetOctomap' (ROS 1)
```


## References
- https://github.com/ros2/ros1_bridge
- https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst
- https://github.com/smith-doug/ros1_bridge/tree/action_bridge_humble
- https://github.com/mjforan/ros-humble-ros1-bridge
- https://packages.ubuntu.com/jammy/ros-desktop-dev -->
