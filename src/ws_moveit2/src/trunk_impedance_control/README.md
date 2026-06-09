# trunk_impedance_control

单电机 CST 力矩模式与 4 轴 trunk EtherCAT 硬件的关节阻抗控制器。

本包包含阻抗控制所需的全部配置、启动文件、URDF/xacro、EtherCAT CST 从站配置与标定脚本。

## 包内目录

```text
trunk_impedance_control/
├── config/          # 控制器、CST 从站、力矩标定参数
├── launch/          # 单电机 / 4 轴 trunk 启动
├── urdf/            # eyou_drive.xacro、trunk_robot.impedance.xacro
├── scripts/         # 力矩 factor 更新、开环阶跃测试
└── src/             # JointImpedanceController 插件
```

位置控制（CSP）仍使用 [`eyou_ethercat_bringup`](../../../eyou_ethercat_bringup) 中的 `bringup.launch.py`。

每个关节的控制律：

```text
τ = K · (q_des - q) + D · (qd_des - qd) + τ_ff + τ_gravity
```

## 单电机（CST）

### 编译

```bash
cd ~/ros2_ethercat_impedance_ws
colcon build --packages-select trunk_impedance_control --symlink-install
source install/setup.bash
```

### 开环力矩标定

根据 EYOU 手册，`0x6071` 与 `0x6077` 的原始单位为 **额定电流的千分比**。ros2_control `effort` 接口使用 **Nm**，换算需 `K_t`（Nm/A）与 `I_rated`（A）：

```text
raw = (I / I_rated) × 1000
I   = τ / K_t
⇒ raw = (τ / (K_t × I_rated)) × 1000

写入 factor = 1000 / (K_t × I_rated)
读取 factor = (K_t × I_rated) / 1000
```

详见 [`config/torque_calibration.yaml`](config/torque_calibration.yaml)。

```bash
ros2 launch trunk_impedance_control single_motor_bringup.launch.py use_effort_controller:=true
```

更新 PDO factor：

```bash
python3 src/ws_moveit2/src/trunk_impedance_control/scripts/update_torque_factors.py \
  --rated-current-a 5.0 --torque-constant-nm-per-a 2.0
colcon build --packages-select trunk_impedance_control --symlink-install
```

阶跃测试：

```bash
bash install/trunk_impedance_control/lib/trunk_impedance_control/run_open_loop_torque_test.sh
ros2 topic echo /joint_states --once
```

### 阻抗控制

```bash
ros2 launch trunk_impedance_control single_motor_bringup.launch.py
```

在线调参：

```bash
ros2 param set /joint_impedance_controller stiffness 2.0
ros2 param set /joint_impedance_controller damping 0.5
ros2 param set /joint_impedance_controller desired_position 0.0
```

### 推荐 K/D 扫参

| 步骤 | K (Nm/rad) | D (Nm·s/rad) | 预期效果 |
|------|-----------|-------------|----------|
| 1 | 0 | 0.5 | 纯阻尼 |
| 2 | 2 | 0.5 | 低刚度弹簧 |
| 3 | 10 | 1.0 | 中等刚度 |

### 急停

```bash
ros2 topic pub --once /estop std_msgs/msg/Empty "{}"
```

## 4 轴 trunk

```bash
ros2 launch trunk_impedance_control trunk_impedance_bringup.launch.py namespace:=trunk_robot
```

切换控制模式：

```bash
ros2 service call /trunk_robot/set_control_mode trunk_teleop_control/srv/SetControlMode "{mode: impedance}"
```

## 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `~/desired_state` | `Float64MultiArray` | `[q_des..., qd_des..., tau_ff...]` |
| `~/impedance_state` | `JointState` | q/qd 与指令力矩 |
| `/estop` | `Empty` | 急停 |

## 控制器切换

```bash
ros2 control switch_controllers --deactivate joint_impedance_controller --activate effort_controller
ros2 control switch_controllers --deactivate effort_controller --activate joint_impedance_controller
```
