# robosense_ac_postprocess

[README](README.md) | [中文文档](README_CN.md)

## 1. 简介

`robosense_ac_postprocess` 是激光与图像的后处理代码仓库，通过图像给点云染色。

## 2. 前置依赖

此项目既支持 ROS1 也支持 ROS2，其中 ROS1 基于 noteic 版本测试通过，ROS2 基于 humble 版本测试通过。

### 2.1 安装  ROS1 或 ROS2

请依照官方说明安装 ROS

* [ROS1 安装](http://wiki.ros.org/noetic/Installation/Ubuntu)

* [ROS2 安装](https://docs.ros.org/en/humble/Installation.html)

## 3. 安装部署

### 3.1 代码拉取

您可以创建一个新的文件夹或进入您现有的工作空间，执行以下命令将代码拉取到工作空间内。

```bash
cd WORKSPACE_PATH/src
mkdir ac_studio && cd ac_studio
git clone https://github.com/RoboSense-Robotics/robosense_ac_postprocess.git -b main
```

### 3.2 安装依赖

可以通过 `rosdep` 工具安装 `robosense_ac_postprocess` 编译所需的依赖

```bash
cd ac_studio
rosdep install --from-paths robosense_ac_postprocess --ignore-src -r -y
```
请先按照[说明](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/blob/main/modules/ac_driver/README.md)编译 ROS2 所需 robosense_msgs，ROS1 则无需编译。
### 3.3 编译 robosense_ac_postprocess

#### 基于 ROS1 环境

在工作空间下执行以下命令来编译安装 `robosense_ac_postprocess`

```
cd WORKSPACE_PATH
catkin build robosense_ac_postprocess
```

编译安装完成后，推荐刷新一下工作空间的 `bash profile`，确保组件功能正常

```
source devel/setup.bash
```

#### 基于 ROS2 环境

```bash
cd WORKSPACE_PATH
colcon build --symlink-install --parallel-workers 8 --packages-select robosense_ac_postprocess
```

编译安装完成后，推荐刷新一下工作空间的 `bash profile`，确保组件功能正常

```bash
source install/setup.bash
```

## 4. 运行
### 4.1 获取数据
可以连接 Active Camera 在线获取数据，或者离线播放数据包进行测试。
#### 4.1.1 运行 Active Camera

参考 [文档](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/tree/main/modules/ac_driver) 运行在线节点，实时获取数据。按照文档说明在终端设置对应环境变量，启用零拷贝模式或非零拷贝模式获取数据。

#### 4.1.2 离线播放数据
使用 `ros` 命令播放数据包，例如：

``` bash
# ros1
rosbag play BAG_PATH
# ros2
ros2 bag play BAG_PATH
```
### 4.2 运行节点

通过 `ros` 命令可以运行节点

```bash
# ros1
roslaunch launch/start.launch
# ros2
ros2 launch robosense_ac_postprocess start.py
```
## 5. FAQ

[Create New Issue](https://github.com/RoboSense-Robotics/robosense_ac_postprocess/issues/new)

## 6. 开源许可

[The Apache License, version 2.0.](https://www.apache.org/licenses/LICENSE-2.0)