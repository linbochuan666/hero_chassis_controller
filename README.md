
# hero_chassis_controller (英雄机器人底盘控制器)

## 一、 概述
本仓库基于 `rm_template` 模板创建，专门用于控制英雄机器人的麦克纳姆轮底盘。该控制器作为 `ros_control` 的标准插件，负责接收上层指令，并通过逆运动学解算向底层硬件下发力矩。

## 二、 依赖项
本项目运行需要以下核心 ROS 依赖包：
- `roscpp`
- `pluginlib` (用于动态加载插件)
- `controller_interface`
- `hardware_interface`

## 三、 编译指南
**注意**：本项目要求使用 `catkin_tools` 进行编译。
在工作空间根目录下执行以下命令：
```bash
catkin build
```
## 四、 运行说明

在启动 Gazebo 仿真或实际机器人时，通过 ControllerManager 读取 controllers.yaml 配置文件，系统会自动通过 pluginlib 动态加载本控制器。

## 五、 开发者信息

· 作者：[林铂川]
· 所属：[]

