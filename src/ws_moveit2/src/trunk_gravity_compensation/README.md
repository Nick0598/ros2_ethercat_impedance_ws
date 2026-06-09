# trunk_gravity_compensation

`trunk_gravity_compensation` is a read-only gravity torque observer for the trunk robot.

Phase 1 scope:

- Subscribe to trunk joint positions from `joint_states`.
- Load the existing trunk URDF from `robot_model/urdf/trunk_robot.urdf`.
- Compute static gravity torque with KDL for `trunk_joint1..4`.
- Publish the computed torque as a `sensor_msgs/msg/JointState` message.
- Do not write any `effort` command to ros2_control or EtherCAT hardware.

## Topics

When launched with the default namespace `trunk_robot`:

- Subscribes: `/trunk_robot/joint_states`
- Publishes: `/trunk_robot/gravity_compensation/torque`

The output message uses:

- `name`: joint names in KDL chain order
- `position`: latest joint positions used for the computation
- `effort`: computed gravity torque in Nm according to the URDF/KDL model

## Launch

Start the normal trunk control stack first so that `/trunk_robot/joint_states` is available. Then run:

```bash
source install/setup.bash
ros2 launch trunk_gravity_compensation gravity_compensation.launch.py namespace:=trunk_robot
```

Check the observer output:

```bash
ros2 topic echo /trunk_robot/gravity_compensation/torque --once
ros2 topic hz /trunk_robot/gravity_compensation/torque
```

For plotting:

```bash
rqt_plot /trunk_robot/gravity_compensation/torque/effort[0] \
  /trunk_robot/gravity_compensation/torque/effort[1] \
  /trunk_robot/gravity_compensation/torque/effort[2] \
  /trunk_robot/gravity_compensation/torque/effort[3]
```

## 重要说明

本包仅观测重力力矩。若要将结果作为电机指令，硬件链路仍需：

- EtherCAT ros2_control xacro 中声明 `effort` 命令接口。
- 将 `0x6071 Target Torque` 映射为 `command_interface: effort`。
- 对 `0x6071` / `0x6077` 做单位标定：EYOU 手册中为 **额定电流的千分比**，需结合 `K_t`（Nm/A）与 `I_rated`（A）换算为 Nm（见 `trunk_impedance_control/config/torque_calibration.yaml`）。
- 验证减速比 / 传动比。
- 控制器仲裁，避免与本节点输出的力矩与轨迹控制器冲突。
