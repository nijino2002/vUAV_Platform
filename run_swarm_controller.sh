#!/usr/bin/env bash
set -e

NET_NAME="vuav-net"
IMAGE="px4-vuav-agent:v5-20251205"

# 保底：网络不存在就建一下
if ! docker network inspect "$NET_NAME" >/dev/null 2>&1; then
  echo "[INFO] creating docker network $NET_NAME ..."
  docker network create "$NET_NAME"
fi

echo "[INFO] 使用镜像: $IMAGE"
echo "[INFO] 在网络: $NET_NAME 上启动 swarm-controller 容器"

docker run --rm -it \
  --name swarm-controller \
  --network "$NET_NAME" \
  -v "$PWD":/app \
  -w /app \
  "$IMAGE" \
  python3 swarm_controller.py
