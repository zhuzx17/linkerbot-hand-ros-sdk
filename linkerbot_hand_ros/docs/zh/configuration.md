# Lifecycle 与参数配置

## Lifecycle 行为

- configure：校验参数，构造设备和 ROS 实体；不打开设备或发送查询。
- activate：打开连接、验证身份并读取 baseline，启动 SDK polling 和命令入口。
- deactivate：停止命令和 polling，保留连接，不自动发送运动或保持目标。
- cleanup/shutdown/error：有界停止 worker 并关闭连接；失败会报告并保留待清理资源以便重试。

launch 默认 `autostart:=true`。手工单手启动（`discover:=false`）仅在启动时自动
configure/activate；首次启动失败或 cleanup 后，再次 configure 会保持 inactive，
需要显式 activate。自动激活必须收到本次 configure 请求的成功响应，不能仅凭状态
事件触发。服务发现及两次响应的总等待上限为 30 秒；失败或结果无法确认时结束
该次自动启动，不重发转换请求。超时不代表服务端未执行，请先检查生命周期状态，
再进行人工管理。外部管理时先启动：

```bash
ros2 launch linkerbot_hand_ros hand.launch.py autostart:=false
```

在另一个已加载环境的终端依次执行所需转换：

```bash
ros2 lifecycle set /linkerbot_hand configure
ros2 lifecycle set /linkerbot_hand activate
ros2 lifecycle set /linkerbot_hand deactivate
ros2 lifecycle set /linkerbot_hand cleanup
```

## 静态参数

configure 后不能修改以下参数，需先 deactivate/cleanup：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `model` | `l6` | `l6/o6/l20lite/l20/o20/l30/o30`，不区分大小写；也接受 `l20-lite` 作为 `L20Lite` 的拼写别名 |
| `side` | `left` | `left/right`；L30 必须为空字符串 |
| `transport_json` | SocketCAN `can0` | 严格 JSON transport 配置 |
| `command_rate_hz` | `250` | SDK 连续命令调度上限 |
| `discrete_queue_capacity` | `16` | 离散和安全配置命令的有界 FIFO 容量 |
| `mailbox_capacity` | `256` | 设备 runtime mailbox 容量 |
| `route_queue_capacity` | `256` | transport 路由队列容量 |
| `identity_timeout_s` | `0.2` | 每个身份响应阶段的等待秒数，包含 L30/O30 后续阶段 |
| `transaction_timeout_s` | `0.0` | `0.0` 沿用各协议默认响应窗口；正数覆盖每个普通响应阶段的等待秒数 |
| `shutdown_timeout_s` | `2.0` | worker 和连接关闭总边界 |
| `auto_reconnect` | `false` | SDK 自动安全恢复 |
| `reconnect_initial_delay_s` | `0.1` | 首次重连退避 |
| `reconnect_max_delay_s` | `2.0` | 最大重连退避 |
| `recovery_max_attempts` | `3` | 每轮恢复最多进行的身份与 baseline 初始化尝试次数，包含首次；正整数 |
| `recovery_timeout_s` | `5.0` | 每轮恢复执行总预算，包含准备、读取与重试等待；正数 |
| `recovery_retry_delay_s` | `0.2` | 响应超时后重试前的最小接收静默间隔；正数 |

型号使用 SDK 规范名称；`product_code` 保留设备实际编号，不能代替 `model`。

## 动态轮询与发布

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `angles_hz` | 100 | 角度反馈目标频率，由反馈策略选择查询/回包 |
| `angle_feedback` | `adaptive` | 角度查询与命令反馈策略 |
| `speeds_hz` | 0 | 实测速度 |
| `force_hz` | 0 | 可用 fn + fns 的频率简写 |
| `force_fn_hz`, `force_fns_hz`, `force_ft_hz`, `force_fts_hz`, `force_fd_hz`, `force_fds_hz`, `force_px_hz` | 0 | 具名触觉字段请求 |
| `temperature_hz`, `current_hz`, `faults_hz`, `voltage_hz` | 0 | 相应实时状态查询 |
| `torques_hz` | 0 | capability 声明的转矩通道；L6 为输出/模式，O30 为实测转矩 |
| `publish_rate_hz` | 100 | 空闲缓存有效性复查；0 暂停状态发布 |
| `diagnostic_rate_hz` | 1 | diagnostics 发布频率 |

非零查询频率须有对应 capability。0 停止该通道的主动查询，不清除缓存，也不阻止命令回显。
`publish_rate_hz=0` 仅停止 ROS 状态发布；SDK 查询仍按 polling plan 运行。
SDK 新缓存会立即唤醒发布，空闲时才按发布频率检查 age/stale；sequence 或有效性变化才发布。

`angle_feedback`：

- `query_only`：主动查询满足角度目标频率；回显仍在 SDK 缓存/指标中，但不发布到 ROS 角度 Topic。
- `command_feedback_only`：只用设备返回的当前角度，不补查询；要求角度命令 `returns_current_state=true`。
- `adaptive`：优先回显，缺少反馈时补查询。

```bash
ros2 param set /linkerbot_hand angles_hz 60.0
ros2 param set /linkerbot_hand angle_feedback query_only
ros2 param set /linkerbot_hand current_hz 10.0
```

触觉通道和共享查询见[触觉指南](force.md)。切换 `force_hz` 与独立字段请求时，用
`AsyncParameterClient.set_parameters_atomically()` 或 `SetParametersAtomically` 服务一次更新。
逐条 `ros2 param set` 不是原子事务。

动态更新经过全部 ROS 校验后才应用。失败会恢复整批参数及运行配置；以最终服务响应判断成功，
参数事件可能包含提议值和恢复值。恢复失败时停止控制并关闭连接，需成功 cleanup 后重新
configure/activate。若错误路径已清理完成，可直接 configure/activate。

参数与 Lifecycle 服务共享互斥回调组；直接从代码调用时也应串行化，参数回调中不支持嵌套更新。
生产入口使用多线程 executor，状态发布由独立缓存变化线程驱动。

## 超时与恢复

`identity_timeout_s` 用于每个身份响应阶段；`transaction_timeout_s=0.0` 映射为 SDK `None`，
沿用协议响应窗口，正数覆盖普通 baseline、读取、轮询及需回复命令的每阶段响应窗口。
计时从该阶段请求批发送完开始，完整回包提前结束等待；不修改协议发送间隔或增加写入重试。

管理服务 `timeout_s` 是调用者等待预算，有限且为正，并须给 `shutdown_timeout_s` 留出 worker
结束余量；它不是响应窗口，也不取消已开始的共享恢复。

每轮恢复最多 `recovery_max_attempts` 次、总预算 `recovery_timeout_s`。只重试身份/baseline
响应超时，不重试身份不匹配、协议错误或写入。重试前取配置间隔、失败响应窗口和协议隔离窗口的最大值，
每个本设备回包都会重新开始静默计时；仍受整轮预算限制。无请求序号的协议不能识别任意迟到帧。
阻塞 I/O 返回后再检查截止；过期恢复不能提交就绪。自动重连可在退避后发起下一轮。

恢复重新验证身份、清除目标并读取 baseline，不重放断线前目标，不自动清错、使能或标定。
连接代次改变后，业务需重新确认反馈并显式提交目标。

## Transport JSON

手工连接格式见[连接指南](connection.md)。未知 backend、未知键、类型错误或缺必填键会被拒绝。

| backend | 必填键 | 可选键 |
| --- | --- | --- |
| `socketcan` | `channel` | `nominal_bitrate`、`data_bitrate`、`auto_reconfigure`、`receive_own_messages` |
| `python_can` | `interface_name`、`interface_type` | `nominal_bitrate`；仅经典 CAN |
| `vendor_canfd` | 无；编号默认 0 | `device_index`、`channel_index`、`library_path`、`nominal_bitrate`、`data_bitrate`、`config_flags`、`model`、`can_type`、`frame_type` |

所有对象都需 `backend`。速率须匹配型号 capability；经典 CAN 不接受 FD 专用选项。
SocketCAN 的布尔选项默认 false。厂商参数默认 `config_flags=7`、`model=0`、`can_type=1`、
`frame_type=4`，按实际适配器和终端接线确认。这里 `model` 是厂商库选项，不是手型号。

## 多设备

一条物理通道一只手；namespace 隔离 ROS 名称，不隔离总线报文。
`multi_hand.launch.py` 在同进程统一 configure 全部设备，再 activate；失败时逆序回滚并报告清理错误。
独立 launch 进程不具备组生命周期管理。

```yaml
/linkerbot_hand_container:
  ros__parameters:
    device_namespaces: [l6_left, o6_right]
    autostart: true

/l6_left/linkerbot_hand:
  ros__parameters:
    model: l6
    side: left
    transport_json: '{"backend":"socketcan","channel":"can0"}'
    angles_hz: 100.0

/o6_right/linkerbot_hand:
  ros__parameters:
    model: o6
    side: right
    transport_json: '{"backend":"socketcan","channel":"can1"}'
    angles_hz: 100.0
```

保存后启动：

```bash
ros2 launch linkerbot_hand_ros multi_hand.launch.py config:=/absolute/path/hands.yaml
```

`device_namespaces` 必须非空且不重复，并有对应节点参数段。`autostart` 默认 true；设 false 时
逐个管理 `/<namespace>/linkerbot_hand`。不同 channel 拥有独立 transport runtime。
需要扫描生成及身份约束时使用[发现部署](discovery.md)，其 YAML 与 ROS 参数文件结构不同。

[返回总览](getting_started.md) · [English](../configuration.md)
