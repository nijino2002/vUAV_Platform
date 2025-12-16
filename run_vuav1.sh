#!/usr/bin/env bash
set -e

IMAGE="px4-vuav-agent:v6-20251215"
NET_NAME="vuav-net"

echo "[INFO] 使用镜像: $IMAGE"
echo "[INFO] 使用网络: $NET_NAME"

docker network inspect "$NET_NAME" >/dev/null 2>&1 || docker network create "$NET_NAME"

cname="vuav1"
echo "[INFO] 停止并删除已有容器: $cname (如果存在)"
docker rm -f "$cname" 2>/dev/null || true

docker run -d \
  --name "$cname" \
  --net "$NET_NAME" \
  -w /px4/PX4-Autopilot/build/px4_sitl_default \
  "$IMAGE" \
  bash -c "./bin/px4 -s etc/init.d-posix/rcS & sleep 5 && python3 /px4/vuav_agent.py"

echo
docker ps --filter name=vuav1
