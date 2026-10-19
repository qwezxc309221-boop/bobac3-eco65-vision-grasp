# 部署与标定

## 1. 前置条件

机器人需具备以下环境：

- Ubuntu 20.04 x86_64
- ROS1 Noetic
- `~/rm_ws/devel/lib/rm_driver/rm_driver`
- `~/bobac3_ws/devel/lib/berxel_camera/berxel_camera`
- Python 3.8

控制器与工控机有线连接，建议单独网段。默认使用 TCP 8080 端口。

## 2. 部署目录

推荐把仓库放在 `~/real_ws`：

```bash
git clone <repo-url> ~/real_ws
cd ~/real_ws
cp machine_config.example.json machine_config.json
```

若使用其他目录，需要同步修改 `machine_config.json` 中的绝对路径。

## 3. 机器配置

| 字段 | 说明 |
|---|---|
| `camera_serial` | 臂载相机 USB 序列号 |
| `camera_namespace` | 相机 ROS 命名空间 |
| `arm_host` / `arm_port` | 控制器 IP 和端口 |
| `host_udp_ip` / `realtime_push_port` | 控制器 UDP 状态上报地址 |
| `speed` | 默认运动速度，0~1 |
| `gripper_force` | 夹爪力度 |
| `gripper_close` | 闭合位置 |
| `weights_dir` | 模型权重目录 |
| `src_dir` | Python 源码目录 |
| `logs_dir` | 日志目录 |
| `calib_offset_file` | 标定偏移文件 |
| `target_file` | 检测目标输出文件 |
| `poses` | 拍照位和中间点位姿 |

### 3.1 识别臂载相机

```bash
lsusb -v -d 0603:0009 2>/dev/null | grep iSerial
```

如果连接了两台同型号相机：

```bash
python3 tools/identify_cameras.py
```

脚本会分别用两个序列号拍照，输出到 `~/real_ws/logs/camera_<序列号>.jpg`。
俯视桌面和工作台的是臂载相机，平视周围环境的是底盘相机。

### 3.2 网络检查

```bash
ping <arm_host>
nc -vz <arm_host> 8080
```

## 4. 安装依赖

```bash
bash scripts/install_deps.sh
```

该脚本安装：

- `torch==2.0.0`、`torchvision==0.15.0`（CPU）
- `ultralytics`、`numpy`、`scipy`、`pillow`
- `third_party/clip` 中的 CLIP 文本编码器

YOLOE 的文本编码器要求 `torch>=2.0`。出厂机常见 `torch 1.13`，需要升级。

## 5. 部署权重

把以下文件放入 `weights/`：

- `yoloe-26s-seg.pt`
- `mobileclip2_b.ts`

然后执行：

```bash
bash scripts/deploy_weights.sh
```

脚本会把文件复制到 `machine_config.json` 的 `weights_dir`。

## 6. 启动环境

```bash
bash scripts/start_ros1_arm.sh
```

启动顺序：

1. roscore（端口 11312）
2. `berxel_camera`，使用配置中的 `camera_serial`
3. `rm_driver`，使用配置中的控制器和 UDP 地址

启动日志位于 `~/real_ws/logs/`。

## 7. 自检

```bash
python3 tools/deploy_check.py
```

ROS 已启动时预期 9 项全部 PASS：

1. `machine_config.json`
2. Python 依赖
3. YOLOE 权重
4. USB 相机序列号
5. 控制器 TCP
6. 相机 serial 参数
7. RGB 话题
8. 深度话题
9. `rm_driver` 节点

若 roscore 未启动，脚本会跳过在线检查并给出提示。

## 8. 冒烟测试

```bash
bash scripts/smoke_test_arm.sh
```

测试内容：

- 环境自愈
- 相机序列号校验
- 控制器状态读取
- J2 关节 +4° 移动并回位
- 夹爪 100 和 8 两个位置的开合

测试移动范围很小，速度 0.05，但仍需确认工作空间安全。

## 9. 标定

抓取点由视觉估计得到，再通过 `calib_offset.json` 补偿安装误差。更换机械臂、
相机支架或重新安装相机后必须重新标定。

### 9.1 记录拍照位

1. 手动把机械臂移到拍照位，相机垂直向下。
2. 读取当前关节角和末端位置：

```bash
python3 src/rm_control.py state
```

3. 把关节角写入 `poses.PHOTO_HOME_DEG`，位置写入 `poses.PHOTO_HOME_XYZ`。

### 9.2 记录中间点

选择从拍照位到抓取点之间、不会碰撞的关节位姿，写入 `poses.INTER_DEG` 和
`poses.INTER_XYZ`。

### 9.3 计算偏移

1. 在拍照位执行检测：

```bash
python3 src/grasp_test.py detect
```

2. 查看 `logs/grasp_target.json` 中的 `px/py/pz`。
3. 手动把夹爪指尖移到检测点对应的实际位置。
4. 读取实际末端位姿，计算实际位置与检测点的差值。
5. 把差值写入 `config/calib_offset.json`：

```json
{"dx": 0.001, "dy": -0.0154, "dz": 0.0241}
```

6. 重复检测和试抓，直到偏差满足要求。

标定文件路径由 `machine_config.json` 的 `calib_offset_file` 指定。程序找不到
文件时会打印 `CALIB_MISSING` 并使用零偏移。

## 10. 运行

```bash
# 完整抓取
bash scripts/run_grasp_work.sh

# 后台抓取
bash scripts/start_grasp_bg.sh

# 只做检测和 IK 解算
python3 src/full_grasp.py --dry-run
```

抓取结果可读取夹爪寄存器：

```bash
python3 src/rm_control.py read 40005   # 位置
python3 src/rm_control.py read 40006   # 扭矩
```

细笔可能已经夹住但位置回读仍接近 8，应结合扭矩和人工确认判断。
