# robosense_ac_postprocess

[README](README.md) | [中文文档](README_CN.md)

## 1. Introduction

`robosense_ac_postprocess` is a post-processing code repository for LiDAR and image data, enabling point cloud coloring based on images.

## 2. Prerequisites

This project supports both ROS1 and ROS2. It has been tested on the Noetic version of ROS1 and the Humble version of ROS2.

### 2.1 Install ROS1 or ROS2

Please follow the official instructions to install ROS:

* [ROS1 Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)

* [ROS2 Installation](https://docs.ros.org/en/humble/Installation.html)

## 3. Installation & Deployment

### 3.1 Pulling the Code

Create a new folder or enter your existing workspace, and execute the following commands to pull the code into the workspace.

```bash
cd WORKSPACE_PATH/src
mkdir ac_studio && cd ac_studio
git clone https://github.com/RoboSense-Robotics/robosense_ac_postprocess.git -b main
```

### 3.2 Installing Dependencies

Use the `rosdep` tool to install the dependencies required for compiling `robosense_ac_postprocess`.

```bash
cd ac_studio
rosdep install --from-paths robosense_ac_postprocess --ignore-src -r -y
```
Please follow the [instructions](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/modules/ac_driver/README.md) to compile the required robosense_msgs for ROS2. Compilation is not needed for ROS1.

### 3.3 Compile robosense_ac_postprocess

#### For ROS1 Environment

Execute the following command in the workspace to compile and install `robosense_ac_postprocess`.

```bash
cd WORKSPACE_PATH
catkin build robosense_ac_postprocess
```

After compilation and installation, it's recommended to refresh the workspace's `bash profile` to ensure component functionality.

```bash
source devel/setup.bash
```

#### For ROS2 Environment

```bash
cd WORKSPACE_PATH
colcon build --symlink-install --parallel-workers 8 --packages-select robosense_ac_postprocess
```

After compilation and installation, it's recommended to refresh the workspace's `bash profile` to ensure component functionality.

```bash
source install/setup.bash
```

## 4. Running
### 4.1 Acquiring Data
Data can be obtained either by connecting to a  Active Camera online or by playing offline data packets for testing.
#### 4.1.1 Running the Super Sensor

Refer to the [documentation](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/tree/main/modules/ac_driver) to run the online node for real-time data acquisition. Follow the documentation instructions to set the corresponding environment variables in the terminal to enable zero-copy mode or non-zero-copy mode for data acquisition.

#### 4.1.2 Offline Data Playback
Use the `ros` command to play back data packets, for example:

```bash
# ROS1
rosbag play BAG_PATH
# ROS2
ros2 bag play BAG_PATH
```
### 4.2 Running Nodes

Run nodes using the `ros` command:

```bash
# ROS1
roslaunch launch/start.launch
# ROS2
ros2 launch robosense_ac_postprocess start.py
```

## 5. FAQ

[Create New Issue](https://github.com/RoboSense-Robotics/robosense_ac_postprocess/issues/new)

## 6. Open Source License

[The Apache License, version 2.0.](https://www.apache.org/licenses/LICENSE-2.0)