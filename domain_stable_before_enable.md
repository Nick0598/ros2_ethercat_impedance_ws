# EtherCAT Domain 稳定后再使能方案说明

## 背景

本工作空间从 Ubuntu 24.04 + ROS 2 Jazzy 迁移到 Ubuntu 22.04 + ROS 2 Humble 后，真实 EtherCAT 电机在开启 DC 同步时出现过如下异常：

- 电机使能后红灯 / Fault。
- `dmesg` 中出现 `Slave did not sync after 5000 ms`、`Fatal Sync Error`、`Sync manager watchdog`。
- EtherCAT Domain Working Counter 逐步变化，例如 `3 -> 6 -> 9 -> 12`，最终可能稳定到 `WC 12`、`State COMPLETE`。
- 关闭 DC 后系统可以启动，控制器也能加载，说明基本 PDO 配置和通信链路大体可用。

进一步测试中发现：如果在 CiA402 自动状态机推进电机使能之前，先等待 EtherCAT Domain 达到稳定状态，可以避免电机报红，并且最终 4 个电机进入 `Operation Enabled`。

## 修改原因

原启动流程中，`EthercatDriver::on_activate()` 在 `EcMaster` 激活后立即开始 `master_->update()`，而 CiA402 插件会在从站进入 EtherCAT OP 后自动推进 Controlword：

- `Switch on Disabled -> Ready to Switch On`
- `Ready to Switch On -> Switch On`
- `Switch On -> Operation Enabled`

这意味着在启动初期 Domain WC 尚未稳定、从站还在陆续进入 OP 的过程中，CiA402 状态机可能已经开始尝试使能电机。对于开启 DC 的场景，这可能导致从站同步还没有稳定时就进入使能流程，从而触发 Fault 或 Sync manager watchdog。

因此本方案的核心目标是：

> EtherCAT 通信可以先正常循环，但在 Domain 连续稳定之前，不允许 CiA402 自动推进 Controlword 到使能状态。

## 设计理念

本次实现采用“门控”方式，而不是停止 EtherCAT 通信：

1. `EcMaster` 激活后继续周期性收发 PDO、同步 DC、刷新从站状态。
2. 启动早期关闭 CiA402 自动状态转换 gate。
3. Controlword 在 gate 关闭时保持 YAML 默认值，例如 `0x6040` 默认值为 `0`，不会自动进入 `0x000F`。
4. 监控 EtherCAT Domain 状态。
5. 当 Domain 连续 `COMPLETE` 且 Working Counter 不变达到指定周期数后，再打开 CiA402 自动状态转换 gate。
6. CiA402 状态机随后正常推进到 `Operation Enabled`。

这样做的好处是：

- 不停止 PDO 输出，避免触发 Sync manager watchdog。
- 不提前使能电机，降低 DC 启动阶段同步不稳导致 Fault 的概率。
- 对非 CiA402 从站影响很小，因为基类 gate 方法默认空实现。

## 稳定判据

当前实现中的稳定条件如下：

- `master_->isDomainComplete()` 返回 true，即 Domain `wc_state == EC_WC_COMPLETE`。
- 当前 Working Counter 与上一个周期相同。
- 连续满足上述条件 `control_frequency_` 个周期。

在当前配置 `control_frequency = 100 Hz` 时，需要连续稳定：

```text
100 cycles = 1 second
```

超时时间设置为：

```text
60 seconds
```

如果 60 秒内没有稳定，硬件激活失败并返回 `CallbackReturn::ERROR`。

## 修改文件与代码位置

### 1. `EcMaster` 增加 Domain 状态查询接口

文件：

```text
src/ethercat_driver_ros2/ethercat_interface/include/ethercat_interface/ec_master.hpp
```

新增声明位置：当前约第 188-189 行。

```cpp
bool isDomainComplete(uint32_t domain = 0) const;
uint32_t getDomainWorkingCounter(uint32_t domain = 0) const;
```

原因：

- 原来的 `checkDomainState()` 只打印 WC 和 COMPLETE/INCOMPLETE 日志。
- 上层 `EthercatDriver::on_activate()` 无法直接判断 Domain 是否稳定。
- 新增只读接口后，上层可以基于已有 `domain_state` 做启动门控。

实现文件：

```text
src/ethercat_driver_ros2/ethercat_interface/src/ec_master.cpp
```

新增实现位置：当前约第 385-401 行。

```cpp
bool EcMaster::isDomainComplete(uint32_t domain) const
{
  const auto iter = domain_info_.find(domain);
  if (iter == domain_info_.end() || iter->second == NULL) {
    return false;
  }
  return iter->second->domain_state.wc_state == EC_WC_COMPLETE;
}

uint32_t EcMaster::getDomainWorkingCounter(uint32_t domain) const
{
  const auto iter = domain_info_.find(domain);
  if (iter == domain_info_.end() || iter->second == NULL) {
    return 0;
  }
  return iter->second->domain_state.working_counter;
}
```

### 2. `EcSlave` 增加 CiA402 gate 的通用空接口

文件：

```text
src/ethercat_driver_ros2/ethercat_interface/include/ethercat_interface/ec_slave.hpp
```

新增位置：当前约第 52 行。

```cpp
virtual void set_auto_state_transitions_gate(bool /*enabled*/) {}
```

原因：

- `EthercatDriver` 持有的是 `std::shared_ptr<ethercat_interface::EcSlave>`。
- 不能在 driver 层直接依赖具体 CiA402 插件类型。
- 通过基类虚函数实现统一调用；非 CiA402 从站默认不做任何事。

### 3. CiA402 插件增加自动状态转换 gate

文件：

```text
src/ethercat_driver_ros2/ethercat_generic_plugins/ethercat_generic_cia402_drive/include/ethercat_generic_plugins/generic_ec_cia402_drive.hpp
```

新增方法声明位置：当前约第 44 行。

```cpp
void set_auto_state_transitions_gate(bool enabled) override;
```

新增成员位置：当前约第 66 行。

```cpp
bool auto_state_transitions_gate_ = true;
```

实现文件：

```text
src/ethercat_driver_ros2/ethercat_generic_plugins/ethercat_generic_cia402_drive/src/generic_ec_cia402_drive.cpp
```

新增方法实现位置：当前约第 31-34 行。

```cpp
void EcCiA402Drive::set_auto_state_transitions_gate(bool enabled)
{
  auto_state_transitions_gate_ = enabled;
}
```

Controlword 自动推进处修改位置：当前约第 79 行。

修改前逻辑：

```cpp
if (auto_state_transitions_) {
  channel.default_value = transition(
    state_,
    channel.ec_read(domain_address));
}
```

修改后逻辑：

```cpp
if (auto_state_transitions_ && auto_state_transitions_gate_) {
  channel.default_value = transition(
    state_,
    channel.ec_read(domain_address));
}
```

原因：

- `auto_state_transitions_` 是配置层面是否启用自动 CiA402 状态机。
- `auto_state_transitions_gate_` 是启动阶段的临时安全门。
- 两者同时为 true 时，才允许自动推进 Controlword。

### 4. `EthercatDriver::on_activate()` 增加稳定等待流程

文件：

```text
src/ethercat_driver_ros2/ethercat_driver/src/ethercat_driver.cpp
```

新增逻辑位置：当前约第 550-615 行。

关键流程：

```cpp
for (auto & module : ec_modules_) {
  module->set_auto_state_transitions_gate(false);
}
```

含义：

- `EcMaster` 激活后，立即关闭所有模块的自动状态转换 gate。
- 对非 CiA402 模块无影响。

稳定周期与 60 秒超时：

```cpp
const uint32_t required_stable_cycles =
  std::max<uint32_t>(1, static_cast<uint32_t>(control_frequency_));
const uint32_t max_startup_cycles =
  std::max<uint32_t>(required_stable_cycles, static_cast<uint32_t>(control_frequency_ * 60.0));
```

含义：

- 需要连续稳定约 1 秒。
- 最长等待 60 秒。

稳定判断：

```cpp
const uint32_t working_counter = master_->getDomainWorkingCounter();
if (master_->isDomainComplete() &&
  has_previous_working_counter &&
  working_counter == previous_working_counter)
{
  ++stable_cycles;
} else {
  stable_cycles = master_->isDomainComplete() ? 1 : 0;
}
```

含义：

- Domain 必须 COMPLETE。
- WC 必须连续不变。
- 中间任何一次不满足都会重新计数。

稳定失败：

```cpp
if (!domain_stable) {
  RCLCPP_ERROR(
    rclcpp::get_logger("EthercatDriver"),
    "EtherCAT domain did not stabilize before CiA402 enable timeout. Last WC: %u.",
    previous_working_counter);
  return CallbackReturn::ERROR;
}
```

稳定成功后放行：

```cpp
for (auto & module : ec_modules_) {
  module->set_auto_state_transitions_gate(true);
}
```

随后进入原来的初始化等待流程：

```cpp
while (running) {
  master_->update();

  bool isAllInit = true;
  for (auto & module : ec_modules_) {
    isAllInit = isAllInit && module->initialized();
  }
  if (isAllInit) {
    running = false;
  }
}
```

## 修改前后的现象对比

### 修改前

开启 DC 后，启动过程中电机可能较早进入 CiA402 使能流程。典型现象：

- 电机红灯 / Fault。
- `Slave did not sync after 5000 ms`。
- `Fatal Sync Error`。
- `Sync manager watchdog`。
- Domain WC 在启动阶段波动。
- 从站进入 `SAFEOP + ERROR`。

### 修改后

测试日志中观察到：

```text
Activated EcMaster!
Waiting for EtherCAT domain to stay COMPLETE for 100 cycles before CiA402 enable (timeout: 60 s).
Domain: WC 3.
Domain: State INCOMPLETE.
Domain: WC 6.
Domain: WC 9.
Domain: WC 12.
Domain: State COMPLETE.
EtherCAT domain stable with WC 12. Releasing CiA402 auto state transitions.
System Successfully started!
Successful 'activate' of hardware 'eyou_trunk_system'
```

随后 CiA402 状态机正常推进：

```text
STATE: Ready to Switch On
STATE: Switch On
STATE: Operation Enabled
```

并且控制器成功加载：

```text
Configured and activated trunk_group_controller
Configured and activated joint_state_broadcaster
```

实际现象：

- 电机未报红。
- Domain 达到 `WC 12`、`State COMPLETE` 后才放行使能。
- 4 个电机进入 `Operation Enabled`。

## 验证命令

启动后可用以下命令确认 EtherCAT 和 ROS2 控制状态：

```bash
ethercat domains
ethercat slaves
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

期望：

- Domain 为 COMPLETE。
- WC 稳定为 12。
- 从站处于 OP。
- `trunk_group_controller` 和 `joint_state_broadcaster` 为 active。
- `/joint_states` 能输出真实状态。

## 适用范围

该方案只对实际加载以下硬件插件的启动链路有效：

```xml
<plugin>ethercat_driver/EthercatDriver</plugin>
```

如果某些 MoveIt demo 或测试 launch 使用的是：

```xml
<plugin>mock_components/GenericSystem</plugin>
```

则不会经过本 EtherCAT driver，也不会触发该稳定等待逻辑。

## 后续建议

当前实现是第一阶段实验版，等待参数是硬编码逻辑：

- 连续稳定周期约 1 秒。
- 超时 60 秒。

如果后续确认长期有效，建议进一步参数化：

```yaml
startup_wait_for_stable_domain: true
startup_stable_seconds: 1.0
startup_stable_timeout_seconds: 60.0
```

也可以进一步记录：

- 稳定前最后一次 WC。
- 稳定耗时。
- 每个从站进入 OP 的时间。
- CiA402 从 `Switch on Disabled` 到 `Operation Enabled` 的耗时。

这些信息有助于后续判断不同机器、不同内核、不同 EtherCAT 主站驱动下的启动稳定性。
