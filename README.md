# BOBAC3 + ECO65 视觉抓取系统

![ROS1 Noetic](https://img.shields.io/badge/ROS1-Noetic-22314E)
![Python 3.8](https://img.shields.io/badge/Python-3.8-3776AB)
![YOLOE](https://img.shields.io/badge/Vision-YOLOE-00A3E0)

基于 ROS1 Noetic 的真机视觉抓取系统，面向 BOBAC3 移动底盘、ECO65 六轴机械臂、
臂载 Berxel 深度相机和 Modbus 电动夹爪。系统完成从 RGB-D 感知、开放词汇目标分割、
三维定位、手眼变换、数值 IK 到机械臂运动与夹爪执行的完整闭环。

> 演示视频：待补充  
> 实测数据：待补充

## 项目亮点

- **开放词汇检测**：使用 YOLOE 文本提示分割，不依赖针对笔类物体的重新训练。
- **RGB-D 三维定位**：融合掩膜深度中值与相机内参，输出基座坐标系下的抓取点。
- **抓取姿态估计**：对分割掩膜做 PCA，估计目标长轴方向，生成绕工具轴的偏航角。
- **数值逆运动学**：基于 `scipy.optimize.least_squares` 的多候选求解，支持关节限位和等效姿态。
- **轨迹安全校验**：对整段 MoveJ 轨迹采样，同时检查法兰和夹爪指尖是否越界。
- **混合控制架构**：运动走 ROS1 `rm_driver` 话题，状态、夹爪和清错走控制器 TCP JSON，
  规避部分控制器状态话题不发布的问题。
- **工程化部署**：配置与代码分离、部署自检、冒烟测试、运行日志和驱动自动恢复。

## 系统架构

```mermaid
flowchart LR
    A[Berxel RGB-D] --> B[YOLOE 分割]
    B --> C[掩膜深度 + PCA]
    C --> D[相机系 3D 点/姿态]
    D --> E[FK + 手眼外参]
    E --> F[基座系抓取位姿]
    F --> G[数值 IK]
    G --> H[工作空间校验]
    H --> I[ROS1 rm_driver]
    I --> J[ECO65]
    J --> K[Modbus 夹爪]
    K --> L[抓取结果/回读]
```

## 硬件与软件

| 项目 | 配置 |
|---|---|
| 移动底盘 | BOBAC3 |
| 机械臂 | ECO65，6 自由度 |
| 相机 | Berxel iHawk100，臂载安装 |
| 夹爪 | Modbus RTU 电动夹爪，115200 |
| 操作系统 | Ubuntu 20.04 x86_64 |
| 中间件 | ROS1 Noetic |
| 视觉模型 | YOLOE + MobileCLIP2 |
| 数学库 | NumPy / SciPy |

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
├── scripts/                        # 启动、部署和自检脚本
├── tools/                          # 独立诊断工具
│   ├── deploy_check.py
│   └── identify_cameras.py
├── docs/                           # 部署、硬件接口、排障文档
├── third_party/clip/               # 离线 CLIP 文本编码器
└── weights/                        # 模型权重（不纳入 Git）
```

## 快速开始

### 1. 部署

推荐把仓库放在机器人的 `~/real_ws`：

```bash
git clone <repo-url> ~/real_ws
cd ~/real_ws
```

### 2. 配置

```bash
cp machine_config.example.json machine_config.json
```

至少修改 `camera_serial`、`arm_host`、`arm_port`、`host_udp_ip`，以及 `weights_dir`、
`src_dir`、`logs_dir` 等绝对路径。

### 3. 安装依赖

```bash
bash scripts/install_deps.sh
```

### 4. 部署权重

从 Releases 下载 `yoloe-26s-seg.pt` 和 `mobileclip2_b.ts`，放入 `weights/` 后执行：

```bash
bash scripts/deploy_weights.sh
```

### 5. 启动与自检

```bash
bash scripts/start_ros1_arm.sh
python3 tools/deploy_check.py
bash scripts/smoke_test_arm.sh
```

### 6. 运行

```bash
# 完整抓取
bash scripts/run_grasp_work.sh

# 只检测、解算，不移动机械臂
python3 src/full_grasp.py --dry-run
```

首次部署或更换机械臂后必须重新标定，步骤见
[部署与标定](docs/DEPLOYMENT.md)。

## 常用命令

| 命令 | 作用 |
|---|---|
| `python3 src/rm_control.py state` | 读取关节角、末端位姿和错误码 |
| `python3 src/grasp_test.py detect` | 检测目标并生成抓取点 |
| `python3 src/full_grasp.py --dry-run` | 只检测和 IK 解算，不动机械臂 |
| `python3 src/grasp_test.py pre` | 开爪并移动到中间点 |
| `python3 src/grasp_test.py down` | 移动到抓取点 |
| `python3 src/grasp_test.py close` | 夹爪闭合 |
| `python3 src/grasp_test.py lift` | 回拍照位提起 |
| `python3 src/rm_control.py read 40005` | 读取夹爪位置 |
| `python3 src/rm_control.py read 40006` | 读取夹爪扭矩 |

## 关键设计

### 1. 运动与状态分离

机械臂运动通过 ROS1 `/rm_driver/MoveJ_Cmd` 和 `/rm_driver/MoveJ_P_Cmd` 下发；
状态读取、夹爪 Modbus 透传和清错则直接连接控制器 TCP JSON。这样即使厂商驱动力
状态话题不可用，抓取流程仍能读取真实状态并完成夹爪控制。

### 2. 检测与三维定位

YOLOE 根据文本类别提示输出检测框和分割掩膜。程序取掩膜内有效深度的中值作为
目标深度，避免单点噪声；再通过相机内参反投影得到相机系三维点。掩膜 PCA 的主方向
用于估计目标长轴，生成绕工具轴的抓取偏航角。

### 3. IK 与安全校验

`ik_solver.py` 使用正运动学残差和 `scipy.optimize.least_squares` 求解关节角，
从多组种子和相差 π 的等效姿态中筛选候选解。`workspace_guard.py` 对候选轨迹按关节
空间采样，同时检查法兰和夹爪指尖是否位于安全框内，并限制抓取过程中的工具倾斜角。

### 4. 标定

视觉点到基座系的变换依赖机械臂 FK 和相机安装外参。`calib_offset.json` 补偿安装
误差和抓取点偏差；拍照位、中间点和标定值都与具体机器相关，换机后必须重新标定。

## 实验结果

> 以下数据请在真机测试后填写，面试时能给出真实数字比空泛描述更有说服力。

| 指标 | 结果 |
|---|---|
| 连续抓取成功率 | 待补充（成功 __ / 总计 __） |
| 单次抓取节拍 | 待补充（__ s） |
| 重复定位误差 | 待补充（±__ mm） |
| 典型失败模式 | 待补充 |

## 已知限制

- 标定值和拍照位与具体机械臂安装相关，换机后需要重新标定。
- 模型权重体积较大，不纳入 Git，需要单独下载。
- 完整流程依赖真实 ROS1 环境和硬件，纯软件环境只能运行数学和配置检查。
- CPU 推理速度受工控机性能影响，实时性要求高时可改用 GPU 或更小模型。

## 文档

- [部署与标定](docs/DEPLOYMENT.md)
- [硬件接口](docs/HARDWARE.md)
- [故障排查](docs/TROUBLESHOOTING.md)

## 许可证

`third_party/clip` 为 Ultralytics 维护的 CLIP 分支，使用 AGPL-3.0 许可证，
分发时请遵守其条款。主仓库尚未指定许可证，发布前请根据使用场景补充。
