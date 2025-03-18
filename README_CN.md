# robosense_ac_postprocess

[README](README.md) | [中文文档](README_CN.md)

## 1. 简介

`robosense_ac_postprocess` 是激光与图像的后处理代码仓库，通过图像给点云染色。

## 2. 前置依赖

此项目基于 `ros2 humble` 进行开发测试

其中 `postprocess_node` 节点依赖 `ros2` 以及 `package.xml` 中定义的依赖项；

### 2.1 安装 ros2

根据您的操作系统选择 [官方教程](https://docs.ros.org/en/humble/Installation.html) 中的指定内容进行执行

## 3. 安装部署

### 3.1 代码拉取

您可以创建一个新的文件夹或进入您现有的 `ros2` 工作空间，执行以下命令将代码拉取到工作空间内，关于工作空间的创建，请参考[官方文档](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)。

```bash
cd WORKSPACE_PATH
mkdir super_sensor_sdk_ros2 && cd super_sensor_sdk_ros2
# http
git clone https://github.com/RoboSense-Robotics/robosense_ac_postprocess.git -b main
```

### 3.2 安装依赖

可以通过 `rosdep` 工具安装 `robosense_ac_postprocess` 编译所需的依赖

```bash
cd super_sensor_sdk_ros2
rosdep install --from-paths robosense_ac_postprocess --ignore-src -r -y
```

### 3.3 编译 robosense_ac_postprocess

在工作空间下执行以下命令来编译安装 `robosense_ac_postprocess`

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
可以连接超级传感器在线获取数据，或者离线播放数据包进行测试。
#### 4.1.1 运行超级传感器

参考 [文档](https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra/tree/main/modules/ros_metas) 运行超级传感器节点，实时获取数据

#### 4.1.2 离线播放数据
使用 `ros2 bag` 命令播放数据包，例如：

``` bash
ros2 bag play BAG_PATH
```
### 4.2 运行节点

通过 `ros2 launch` 命令可以运行节点

```bash
ros2 launch robosense_ac_postprocess start.py
```
## 5. FAQ

[Create New Issue](https://github.com/RoboSense-Robotics/robosense_ac_postprocess/issues/new)

## 6. 开源许可

[The Apache License, version 2.0.](https://www.apache.org/licenses/LICENSE-2.0)