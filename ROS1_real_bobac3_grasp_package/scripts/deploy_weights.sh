#!/bin/bash
# 把包内 weights/ 部署到 machine_config.json 指定的 weights_dir
set -e
MCFG="$(cd "$(dirname "$0")/.." && pwd)/machine_config.json"
WDIR=$(python3 -c "import json;print(json.load(open('$MCFG'))['weights_dir'])")
PKG_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$PKG_ROOT/weights"

if [ ! -f "$SRC/yoloe-26s-seg.pt" ] || [ ! -f "$SRC/mobileclip2_b.ts" ]; then
  echo "[weights] 包内缺少权重文件：$SRC"
  exit 1
fi

mkdir -p "$WDIR"
cp -f "$SRC/yoloe-26s-seg.pt" "$SRC/mobileclip2_b.ts" "$WDIR/"
ls -la "$WDIR"
echo "[weights] OK -> $WDIR"
