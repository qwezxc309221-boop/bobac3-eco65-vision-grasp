# 硬件接口

## 1. ROS1 话题

| 功能 | 话题 | 消息类型 | 说明 |
|---|---|---|---|
| 关节运动 | `/rm_driver/MoveJ_Cmd` | `rm_msgs/MoveJ` | 关节角为弧度，speed 0~1 |
| 位姿运动 | `/rm_driver/MoveJ_P_Cmd` | `rm_msgs/MoveJ_P` | 位置为米，姿态为四元数 |
| 直线运动 | `/rm_driver/MoveL_Cmd` | `rm_msgs/MoveL` | 位置和姿态格式同上 |
| 状态触发 | `/rm_driver/GetArmState_Cmd` | `rm_msgs/GetArmState_Command` | 请求状态 |
| 状态回传 | `/rm_driver/ArmCurrentState` | `rm_msgs/ArmState` | 部分控制器不发布 |
| 关节使能/清错 | `/rm_driver/Joint_Enable` | 厂商消息 | 关节号 1~6 |
| 系统清错 | `/rm_driver/Clear_System_Err` | `std_msgs/Empty` | 清除系统错误 |

本仓库的运动指令统一走 `MoveJ_Cmd`。状态读取不依赖 ROS 话题，避免驱动状态
话题缺失导致流程中断。

## 2. 控制器 TCP JSON

默认端口 8080，每条命令一行 JSON，以 `\r\n` 结尾。

### 2.1 读取状态

```json
{"command":"get_current_arm_state"}
```

返回的 `arm_state.joint` 单位为 0.001 度，`arm_state.pose` 前三项单位为
微米，后三项单位为 0.001 弧度。`rm_control.py` 会转换为度和米。

### 2.2 清错

```json
{"command":"set_joint_clear_err","joint_clear_err":1}
{"command":"clear_system_err"}
```

`joint_clear_err` 依次发送 1~6，再发送系统清错。

### 2.3 夹爪初始化

```json
{"command":"set_tool_voltage","voltage_type":3}
{"command":"set_modbus_mode","port":1,"baudrate":115200,"timeout":100}
{"command":"write_single_register","port":1,"address":40010,"data":50,"device":1}
```

### 2.4 夹爪读写

```json
{"command":"write_single_register","port":1,"address":40000,"data":100,"device":1}
{"command":"read_holding_registers","port":1,"address":40005,"device":1}
```

## 3. 夹爪寄存器

| 地址 | 含义 | 说明 |
|---|---|---|
| 40000 | 目标位置 | 0~100，100 为张开，8 为闭合 |
| 40001 | 力度 | 0~100 |
| 40005 | 当前位置回读 | 空抓通常 7~8，夹住后增大 |
| 40006 | 扭矩回读 | 大于 0 表示存在夹持力 |
| 40010 | 初始化寄存器 | 初始化时写 50 |

## 4. 坐标变换

视觉点到基座系的变换链：

```text
base
  -> link_6           正运动学，参数见 workspace_guard.py
  -> eco65_end_link   单位变换
  -> Cam_base_link    静态外参，见 camera_to_base.py
  -> depth_frame      相机 TF
```

夹爪长度按 0.166 m 计算，抓取点在目标点沿工具轴方向偏移该长度。

## 5. 工作空间

`config/workspace_limits.json` 定义软件安全框，单位为米：

```json
{
  "x_min": -0.70,
  "x_max": 0.55,
  "y_min": -0.55,
  "y_max": 0.55,
  "z_min": 0.03,
  "z_max": 0.95
}
```

校验对象包括 link6 和夹爪指尖。对 MoveJ 轨迹按关节空间线性采样，任意采样点
越界都会拒绝执行。

## 6. 速度与力度

- 默认运动速度 `0.1`，对应控制器速度比例的 10%。
- 冒烟测试使用 `0.05`。
- 夹爪力度默认 80，闭合位置 8。

修改速度或力度前先确认负载和抓取对象，避免夹伤或碰撞。
