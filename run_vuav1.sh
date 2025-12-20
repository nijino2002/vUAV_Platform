#!/usr/bin/env bash
set -euo pipefail

IMAGE="px4-vuav-agent:v6-20251215"
NET_NAME="vuav-net"
cname="vuav1"

echo "[INFO] 使用镜像: $IMAGE"
echo "[INFO] 使用网络: $NET_NAME"

docker network inspect "$NET_NAME" >/dev/null 2>&1 || docker network create "$NET_NAME"

echo "[INFO] 停止并删除已有容器: $cname (如果存在)"
docker rm -f "$cname" 2>/dev/null || true

docker run -d \
  --name "$cname" \
  --net "$NET_NAME" \
  --init \
  -e PX4_SYS_AUTOSTART=10040 \
  -w /px4/PX4-Autopilot/build/px4_sitl_default \
  "$IMAGE" \
  bash -lc '
    set -euo pipefail

    echo "[entry] start PX4 (background)..."
    ./bin/px4 -s etc/init.d-posix/rcS 0 &

    # 给 PX4 启动一点时间（agent 里也会 wait_heartbeat）
    sleep 2

    echo "[entry] start vuav_agent (foreground PID1)..."
    exec python3 /px4/vuav_agent.py
  '

echo
docker ps --filter name="$cname"
echo
echo "[INFO] 查看日志：docker logs -f $cname"
