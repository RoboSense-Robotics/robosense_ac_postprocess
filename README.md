# robosense_ac_postprocess

[README](README.md) | [中文文档](README_CN.md)

## 1. Introduction

`robosense_ac_postprocess` is a repository for post-processing LiDAR and image data, coloring point clouds based on images.

## 2. Prerequisites

This project is developed and tested based on `ros2 humble`.

The `postprocess_node` depends on `ros2` and the dependencies defined in `package.xml`.

### 2.1 Install ROS2

Follow the designated content in the [official guide](https://docs.ros.org/en/humble/Installation.html) according to your operating system.

## 3. Installation & Deployment

### 3.1 Code Checkout

Create a new folder or navigate to your existing `ros2` workspace, then execute the following commands to pull the code into the workspace. For creating a workspace, refer to the [official documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

```bash
cd WORKSPACE_PATH
mkdir ac_studio && cd ac_studio
git clone https://github.com/RoboSense-Robotics/robosense_ac_postprocess.git -b main
```

### 3.2 Install Dependencies

You can use the `rosdep` tool to install the dependencies needed for compiling `robosense_ac_postprocess`:

```bash
cd ac_studio
rosdep install --from-paths robosense_ac_postprocess --ignore-src -r -y
```

### 3.3 Build robosense_ac_postprocess

Execute the following command under your workspace to compile and install `robosense_ac_postprocess`:

```bash
cd WORKSPACE_PATH
colcon build --symlink-install --parallel-workers 8 --packages-select robosense_ac_postprocess
```

After compilation and installation, it's recommended to refresh the workspace's `bash profile` to ensure component functionality:

```bash
source install/setup.bash
```

## 4. Running

### 4.1 Acquire Data

Data can be obtained online from Super Sensors or offline through data packets for testing.

#### 4.1.1 Running Super Sensors

Refer to the [documentation](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/tree/main/modules/ros_metas) to run the Super Sensor node for real-time data acquisition.

#### 4.1.2 Offline Playback

Use the `ros2 bag` command to play back data packets, for example:

```bash
ros2 bag play BAG_PATH
```

### 4.2 Running Nodes

The node can be launched using the `ros2 launch` command:

```bash
ros2 launch robosense_ac_postprocess start.py
```

## 5. FAQ

[Create New Issue](https://github.com/RoboSense-Robotics/robosense_ac_postprocess/issues/new)

## 6. Open Source License

[The Apache License, version 2.0.](https://www.apache.org/licenses/LICENSE-2.0)