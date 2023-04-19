# Visual SLAM, Odometry and Mapping 

## Overview

This module contains the ROS2 source code for:
- Isaac Visual SLAM 
- RTAM-mapping.

These are run into a container provided by Isaac Visual SLAM.

## Isaac Sim

The scripts in this folder can have input from either real sensors attached to a robot or from a simulated environment such as Isaac Sim. To start Isaac Sim follow the instructions in [this repository]().

### Docker Bringup

1. Go to the correct folder:

```bash
cd ~/ROB10_repos/isaac_rover_physical_2.0/isaac_ros-dev/
```
2. Start the container:

> **Note** : This takes the form:
>```bash
>scripts/run_dev.sh <path to workspace>
>```
In our specific case, the command is:

```bash
./src/isaac_ros_common/scripts/run_dev.sh ~/ROB10_repos/isaac_rover_physical_2.0/isaac_ros-dev/
```
At this point the docker shell should be active.

3. Setup the ROS2 environment:
> **Optional** : Run the colcon build for all the ROS2 packages used.
>```bash
>rosdep update && rosdep install --from-paths src --ignore-src -r -y
>colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release
>```

```bash
export ROS_DOMAIN_ID=5
source install/setup.bash
```
> **Optional** : Check if ROS2 works by listing the topics it sees:
>```bash
>ros2 topic list
>```

### Run Isaac ROS Visual SLAM

Run Isaac Sim and then follow steps 9 to 11 from [this tutorial](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/blob/main/docs/tutorial-isaac-sim.md#tutorial-with-isaac-sim).

### Run RTAB map

1. Run Isaac Sim. 
2. Launch RTAM mapping:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py visual_odometry:=false frame_id:=base_link odom_topic:=/odom args:="-d" use_sim_time:=true rgb_topic:=/rgb_right depth_topic:=/depth_right camera_info_topic:=/camera_info_right approx_sync:=true qos:=2 rviz:=true
```
If the commannd fails, it is likely that the RTAB packages are not built. To solve this, run Step 3 of the Docker Bringup with the **Optional** commands as well.