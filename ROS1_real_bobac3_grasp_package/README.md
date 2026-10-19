# ROS1 BOBAC3 / ECO65 抓取包

基于 ROS1 Noetic 的真机抓取程序，面向 BOBAC3 底盘、ECO65 六轴机械臂、
臂载 Berxel 深度相机和 Modbus 电动夹爪。

视觉部分使用 YOLOE 开放词汇分割检测笔类物体，结合深度图得到三维目标点，
经手眼外参变换和数值 IK 生成关节角；机械臂运动通过 ROS1 `rm_driver` 话题下发，
状态读取、夹爪控制和清错通过控制器 TCP JSON 接口完成。

## 功能

- 笔类物体检测、分割和抓取位姿估计
- 相机坐标系到机械臂基座坐标系的变换
- 基于 `scipy` 的数值 IK，支持多候选解和安全路径检查
- 工作空间软件限位，拦截超界轨迹
- 一键抓取、分步调试、无运动干跑
- 部署自检和冒烟测试

## 仓库结构

```text
.
├── README.md
├── requirements.txt
├── machine_config.example.json     # 机器配置模板
├── config/
│   ├── calib_offset.example.json   # 抓取偏移模板
│   └── workspace_limits.json       # 工作空间限位
├── src/                            # Python 实现
│   ├── rm_control.py               # 机械臂/夹爪/状态控制层
│   ├── grasp_test.py               # 检测与分步调试
│   ├── full_grasp.py               # 完整抓取流程（支持 --dry-run）
│   ├── grasp_work.py               # 生产抓取入口
│   ├── ik_solver.py                # 数值逆运动学
│   ├── camera_to_base.py           # 相机系到基座系变换
│   └── workspace_guard.py          # 正运动学与工作空间校验
├── scripts/                        # Shell 启动和部署脚本
├── tools/                          # 独立诊断工具
│   ├── deploy_check.py
│   └── identify_cameras.py
├── docs/                           # 部署、硬件接口、排障文档
├── third_party/clip/               # 离线 CLIP 文本编码器
└── weights/                        # 模型权重（不纳入 Git）
```

## 环境要求

- Ubuntu 20.04 x86_64
- ROS1 Noetic（`/opt/ros/noetic`）
- `~/rm_ws`：已编译的 ROS1 `rm_driver`
- `~/bobac3_ws`：已编译的 `berxel_camera`
- Python 3.8
- 机械臂控制器与工控机同网段，8080 端口可达

不需要 Docker，不需要 ROS2。若机器同时装有 ROS2，运动仍统一走 ROS1 驱动。

## 快速开始

### 1. 部署

将仓库放到机器人上，推荐目录为 `~/real_ws`：

```bash
git clone <repo-url> ~/real_ws
cd ~/real_ws
```

### 2. 配置

```bash
cp machine_config.example.json machine_config.json
```

至少修改以下字段：

- `camera_serial`：臂载相机 USB 序列号
- `arm_host`、`arm_port`：控制器地址
- `host_udp_ip`：工控机有线网卡地址
- `weights_dir`、`src_dir`、`logs_dir`、`calib_offset_file`、`target_file`

相机序列号可用以下命令查看：

```bash
lsusb -v -d 0603:0009 2>/dev/null | grep iSerial
```

无法区分两台相机时，运行 `python3 tools/identify_cameras.py`，对比两张照片后
把臂载相机的序列号写入配置。

### 3. 安装依赖

```bash
bash scripts/install_deps.sh
```

脚本会安装 `torch 2.0 CPU`、`torchvision 0.15`、`ultralytics`、`numpy`、
`scipy`、`pillow`，并从 `third_party/clip` 离线安装 CLIP 文本编码器。

### 4. 部署权重

将 `yoloe-26s-seg.pt` 和 `mobileclip2_b.ts` 放入 `weights/`，然后执行：

```bash
bash scripts/deploy_weights.sh
```

权重文件较大，默认不提交到 Git，详见 `weights/README.md`。

### 5. 启动 ROS 环境

```bash
bash scripts/start_ros1_arm.sh
```

脚本按配置启动 roscore（端口 11312）、臂载相机和 `rm_driver`。

### 6. 自检与冒烟测试

```bash
python3 tools/deploy_check.py
bash scripts/smoke_test_arm.sh
```

自检覆盖配置、依赖、权重、相机序列号、控制器 TCP、相机话题和驱动节点。
冒烟测试会让 J2 关节移动 +4° 后回位，并测试夹爪开合。

### 7. 标定

抓取点与机械臂安装位置相关。首次部署或更换机械臂后必须重新标定，
否则检测正常但抓取会偏移。步骤见 [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)。

## 使用

```bash
# 完整抓取
bash scripts/run_grasp_work.sh

# 只检测、解算，不移动机械臂
python3 src/full_grasp.py --dry-run

# 只检测并保存标注图
python3 src/grasp_test.py detect

# 分步执行
python3 src/grasp_test.py pre
python3 src/grasp_test.py down
python3 src/grasp_test.py close
python3 src/grasp_test.py lift
```

常用产物：

| 文件 | 内容 |
|---|---|
| `logs/vision_debug_boxed.jpg` | 检测框、掩膜和中心点标注图 |
| `logs/grasp_target.json` | 目标点、姿态、置信度和深度 |
| `logs/grasp_work.log` | 抓取流程日志 |
| `logs/full_grasp_history.log` | 抓取结果历史 |

## 控制接口

| 环节 | 接口 |
|---|---|
| 机械臂运动 | ROS1 `/rm_driver/MoveJ_Cmd`、`/rm_driver/MoveJ_P_Cmd` |
| 机械臂状态 | TCP JSON `get_current_arm_state` |
| 清错 | TCP JSON `set_joint_clear_err`、`clear_system_err` |
| 夹爪 | TCP JSON Modbus 透传，寄存器 40000/40001/40005/40006 |

详细话题、消息格式和寄存器说明见 [docs/HARDWARE.md](docs/HARDWARE.md)。

## 安全

- 首次运行先做自检和冒烟测试，确认急停按钮可触及。
- 调试阶段使用 `speed: 0.1`，单次测试移动不超过 5°。
- 抓取前确认工作空间内无人员和障碍物。
- 出现碰撞错误（4109）时先清错并排查障碍物，不要反复下发运动指令。
- 软件限位不能替代急停和物理安全措施。

## 文档

- [部署与标定](docs/DEPLOYMENT.md)
- [硬件接口](docs/HARDWARE.md)
- [故障排查](docs/TROUBLESHOOTING.md)

## 许可证

`third_party/clip` 为 Ultralytics 维护的 CLIP 分支，使用 AGPL-3.0 许可证，
分发时请遵守其条款。主仓库尚未指定许可证，发布前请根据使用场景补充。
