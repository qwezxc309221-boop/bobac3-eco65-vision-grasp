#!/bin/bash
# setup_and_deploy.sh - 一键部署与自检
#
# 用法:
#   bash setup_and_deploy.sh            # 部署 + 自检
#   bash setup_and_deploy.sh --smoke    # 部署 + 自检 + 冒烟测试（会小范围移动机械臂）
#
# 前提：先修改包根目录 machine_config.json（相机序列号/控制器IP/UDP IP/路径）
set -u

PKG_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MCFG="$PKG_ROOT/machine_config.json"
SCRIPTS="$PKG_ROOT/scripts"
SRC="$PKG_ROOT/src"
TOOLS="$PKG_ROOT/tools"
PASS=0; FAIL=0; WARN=0
ok(){ echo "[ OK ] $1"; PASS=$((PASS+1)); }
warn(){ echo "[WARN] $1"; WARN=$((WARN+1)); }
fail(){ echo "[FAIL] $1"; FAIL=$((FAIL+1)); }

echo "===== ROS1 抓取包一键部署 ====="
echo "包目录: $PKG_ROOT"

# 0 包完整性
[ -f "$MCFG" ] || { fail "缺少 machine_config.json（从机器配置模板复制并修改）"; exit 1; }
[ -d "$SCRIPTS" ] || { fail "缺少 scripts/"; exit 1; }
[ -d "$SRC" ] || { fail "缺少 src/"; exit 1; }
[ -d "$TOOLS" ] || { fail "缺少 tools/"; exit 1; }
ok "包结构完整"

# 1 出厂软件栈检查
if source /opt/ros/noetic/setup.bash 2>/dev/null; then ok "ROS1 Noetic 可用"; else fail "未找到 /opt/ros/noetic"; fi
if [ -x "$HOME/rm_ws/devel/lib/rm_driver/rm_driver" ]; then ok "rm_ws/rm_driver 存在"; else fail "缺少 ~/rm_ws/devel/lib/rm_driver/rm_driver"; fi
if [ -x "$HOME/bobac3_ws/devel/lib/berxel_camera/berxel_camera" ]; then ok "bobac3_ws/berxel_camera 存在"; else fail "缺少 ~/bobac3_ws/devel/lib/berxel_camera/berxel_camera"; fi

# 2 配置校验
CAM=$(python3 -c "import json;print(json.load(open('$MCFG'))['camera_serial'])" 2>/dev/null || echo "")
ARM=$(python3 -c "import json;print(json.load(open('$MCFG'))['arm_host'])" 2>/dev/null || echo "")
PORT=$(python3 -c "import json;print(json.load(open('$MCFG'))['arm_port'])" 2>/dev/null || echo "8080")
[ -n "$CAM" ] && ok "machine_config camera_serial=$CAM" || fail "machine_config 缺少 camera_serial（先改配置文件）"
[ -n "$ARM" ] && ok "machine_config arm_host=$ARM:$PORT" || fail "machine_config 缺少 arm_host（先改配置文件）"

USB=$(lsusb -v -d 0603:0009 2>/dev/null | grep iSerial | awk '{print $NF}' | tr '\n' ' ')
if [ -n "$CAM" ]; then
  case " $USB " in
    *" $CAM "*) ok "臂载序列号 $CAM 已在 USB 列表" ;;
    *) warn "配置序列号 $CAM 不在当前 USB 列表 [$USB]（请核对/修改 machine_config.json）" ;;
  esac
fi
if [ -n "$ARM" ]; then
  if timeout 3 bash -c "echo > /dev/tcp/$ARM/$PORT" 2>/dev/null; then
    ok "控制器 TCP $ARM:$PORT 可连"
  else
    warn "控制器 TCP $ARM:$PORT 不可达（检查网线/控制器 IP/上电）"
  fi
fi

# 3 Python 依赖（torch>=2.0 + ultralytics + clip 全满足则跳过，缺任一自动补装）
if python3 -c "import torch; assert int(torch.__version__.split('.')[0])>=2; import ultralytics, clip" 2>/dev/null; then
  ok "Python 依赖已满足（torch $(python3 -c "import torch;print(torch.__version__)") / ultralytics / clip）"
else
  echo "[----] 安装/补装 Python 依赖（torch>=2.0 / ultralytics / CLIP，约几分钟）..."
  if bash "$SCRIPTS/install_deps.sh"; then ok "Python 依赖安装完成"; else fail "Python 依赖安装失败（看上方日志）"; fi
fi

# 4 权重部署
if [ -f "$PKG_ROOT/weights/yoloe-26s-seg.pt" ] && [ -f "$PKG_ROOT/weights/mobileclip2_b.ts" ]; then
  if bash "$SCRIPTS/deploy_weights.sh"; then ok "权重部署完成"; else fail "权重部署失败"; fi
else
  warn "包内未发现权重文件，跳过（请手动部署到 machine_config 的 weights_dir）"
fi

# 5 启动环境
echo "[----] 启动 roscore/臂载相机/rm_driver..."
if bash "$SCRIPTS/start_ros1_arm.sh"; then ok "环境启动完成"; else fail "环境启动失败（看 ~/real_ws/logs/*.log）"; fi

# 6 部署自检
echo "[----] 部署自检（期望 9/9 PASS）..."
if python3 "$TOOLS/deploy_check.py"; then ok "deploy_check 通过"; else fail "deploy_check 存在 FAIL（看上面输出）"; fi

# 7 可选冒烟
if [ "${1:-}" = "--smoke" ]; then
  echo "[----] 冒烟测试（小范围移动 + 夹爪）..."
  if bash "$SCRIPTS/smoke_test_arm.sh"; then ok "冒烟测试通过"; else fail "冒烟测试失败"; fi
fi

echo "===== 部署总结: PASS=$PASS FAIL=$FAIL WARN=$WARN ====="
echo "下一步：重新标定（docs/DEPLOYMENT.md）→ bash $SCRIPTS/run_grasp_work.sh"
[ "$FAIL" -eq 0 ] && exit 0 || exit 1
