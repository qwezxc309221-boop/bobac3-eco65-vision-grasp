#!/bin/bash
# Python 依赖安装（ROS1 抓取包）
# 注意：YOLOE 文本编码器要求 torch>=2.0，出厂机通常只有 torch 1.13，必须升级
set -e
PKG_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "[deps] 安装 torch 2.0 CPU + torchvision 0.15（YOLOE 必需）"
pip install torch==2.0.0 torchvision==0.15.0 --index-url https://download.pytorch.org/whl/cpu

echo "[deps] 安装/升级 Python 依赖"
pip install "ultralytics>=8.3" numpy scipy pillow

echo "[deps] 安装 ultralytics CLIP 文本编码器（YOLOE 必需）"
if [ -d "$PKG_ROOT/third_party/clip" ]; then
  pip install "$PKG_ROOT/third_party/clip"
else
  echo "[deps] 包内未找到 third_party/clip，尝试从 GitHub 安装..."
  pip install "git+https://github.com/ultralytics/CLIP.git" || \
    echo "[deps] WARN: CLIP 安装失败，检测时会卡在 AutoUpdate，请手动处理"
fi

echo "[deps] 验证"
python3 -c "import torch, torchvision, ultralytics, clip; print('torch', torch.__version__, '| torchvision', torchvision.__version__, '| ultralytics', ultralytics.__version__, '| clip OK')"
echo "[deps] OK"
