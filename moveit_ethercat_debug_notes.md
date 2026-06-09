# MoveIt + EtherCAT 调试记录

## 1. JointTrajectory 带速度字段导致电机报红

### 现象

通过 topic 向 `trunk_group_controller` 发送轨迹命令时，如果包含 `velocities` 字段，部分或全部电机进入 Fault。

驱动器错误码：

```text
0x8611
```

电机手册含义：位置偏差过大。

### 原因

`/joint_states` 中的 `velocity` 数值异常，例如：

```yaml
velocity:
  - 8000.0
  - -8000.0
```

这些值是驱动器原始速度值，未转换为 ROS 使用的 `rad/s`。当轨迹命令中包含 `velocities` 字段时，`joint_trajectory_controller` 会使用错误的当前速度进行插补，可能导致 CSP 目标位置跳变，引发位置偏差过大。

相关配置：

```yaml
- {index: 0x606C, sub_index: 0, type: int32, state_interface: velocity}
```

涉及文件：

```text
src/eyou_ethercat_bringup/config/eyou_slave.yaml
src/eyou_ethercat_bringup/config/eyou_slave_reverse.yaml
```

### 解决办法

临时规避：发送 `JointTrajectory` 命令时不要填写 `velocities` 字段，只发送 `positions` 和 `time_from_start`。

长期修复：根据电机手册为 `0x606C` 增加正确的速度换算系数。如果 `0x606C` 单位是 encoder counts/s，可先参考 position factor；反向轴使用负号。

示例：

```yaml
- {index: 0x606C, sub_index: 0, type: int32, state_interface: velocity, factor: 9.904318103600473e-08}
```

反向轴：

```yaml
- {index: 0x606C, sub_index: 0, type: int32, state_interface: velocity, factor: -9.904318103600473e-08}
```

## 2. move_group 因 request_adapters 类型错误崩溃

### 现象

启动 `demo.launch.py` 时，`move_group` 退出：

```text
move_group process has died, exit code -6
```

日志中出现：

```text
parameter 'ompl.request_adapters' has invalid type: expected [string] got [string_array]
```

### 原因

ROS 2 Humble / 当前 MoveIt 版本要求 `request_adapters` 是字符串，而不是 YAML 数组。

### 代码改动

修改文件：

```text
src/ws_moveit2/src/trunk_configure/config/ompl_planning.yaml
```

将数组形式改为字符串形式：

```yaml
request_adapters: >-
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planner_request_adapters/ResolveConstraintFrames
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints
```

## 3. RViz 中只显示 CHOMP，看不到 RRTConnect

### 现象

RViz MotionPlanning 面板中，Planner 区域只显示 `CHOMP` / `<unspecified>`，看不到 `RRTConnectkConfigDefault`。

### 原因

`ompl_planning.yaml` 中 OMPL 插件参数名不正确。当前 Humble 配置应使用 `planning_plugin`，不是 `planning_plugins`。

### 代码改动

修改文件：

```text
src/ws_moveit2/src/trunk_configure/config/ompl_planning.yaml
```

修改为：

```yaml
planning_plugin: ompl_interface/OMPLPlanner
```

修改后重新编译并 source：

```bash
colcon build --packages-select trunk_configure
source install/setup.bash
```
