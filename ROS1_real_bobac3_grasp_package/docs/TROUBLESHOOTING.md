# 故障排查

## 常见问题

| 现象 | 可能原因 | 处理 |
|---|---|---|
| `Unknown builtin op: aten::scaled_dot_product_attention` | torch 低于 2.0 | `bash scripts/install_deps.sh` |
| 抓取第 0 步退出 | `rm_driver` 不可用 | 运行 `scripts/start_ros1_arm.sh`，或检查 `logs/rm_driver.log` |
| `NO_BOX` | 目标不在视野、置信度低、反光 | 调整摆放，查看 `logs/vision_debug_boxed.jpg` |
| 检测框正确但抓空 | 标定或拍照位姿不准 | 重新标定 `config/calib_offset.json` 和 `poses` |
| 相机话题无消息 | 相机进程退出或 USB 被占用 | 重新运行 `scripts/start_ros1_arm.sh` |
| `SMOKE_FAIL camera serial` | 配置序列号与实机不一致 | 重新识别并修改 `camera_serial` |
| 控制器 TCP 不可达 | 网线、IP、端口或控制器未上电 | `ping` 和 `nc -vz <ip> 8080` |
| 夹爪回读 40005 仍为 8 | 空抓或细笔 | 结合 40006 扭矩和人工确认 |
| 4109 碰撞错误 | 路径上有障碍物或位姿错误 | 先清错，检查工作空间后再运行 |
| `CALIB_MISSING` | 标定文件路径不存在 | 创建 `calib_offset.json` 或修改 `calib_offset_file` |

## 日志位置

默认日志目录为 `~/real_ws/logs`：

| 文件 | 内容 |
|---|---|
| `roscore.log` | ROS master |
| `camera_arm.log` | 相机驱动 |
| `rm_driver.log` | 机械臂驱动 |
| `grasp_work.log` | 抓取流程 |
| `detect.log` | 后台检测 |
| `vision_debug_boxed.jpg` | 检测标注图 |
| `grasp_target.json` | 最近一次检测目标 |
| `full_grasp_history.log` | 抓取结果历史 |

## 驱动不稳定

部分控制器固件与厂商 ROS1 `rm_driver` 存在兼容问题，驱动可能在运行几分钟后
退出。`rm_control.ensure_driver()` 会在每次运动前检查进程和 TCP 连通性，
不可用时自动重启。若频繁崩溃，检查：

```bash
tail -n 100 ~/real_ws/logs/rm_driver.log
rosnode list
pgrep -af rm_driver
```

状态读取和夹爪控制不依赖该驱动，驱动退出不会影响已下发的运动执行。

## 相机序列号

查看当前 USB 设备：

```bash
lsusb -v -d 0603:0009 2>/dev/null | grep iSerial
```

查看 ROS 参数中的序列号：

```bash
rosparam get /bobac3_arm/serial_no
```

两者不一致时修改 `machine_config.json` 的 `camera_serial`，然后重启相机。

## 重新开始

```bash
pkill -f 'devel/lib/rm_driver/rm_driver' || true
pkill -f 'devel/lib/berxel_camera/berxel_camera' || true
pkill -f roscore || true
bash scripts/start_ros1_arm.sh
python3 tools/deploy_check.py
```

执行前确认机械臂处于安全位置。
