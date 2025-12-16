#!/usr/bin/env bash
set -e

IMAGE="px4-vuav-agent:v5-20251205"
NET_NAME="vuav-net"

echo "[INFO] 使用镜像: $IMAGE"
echo "[INFO] 使用网络: $NET_NAME"

# 确保网络存在
docker network inspect "$NET_NAME" >/dev/null 2>&1 || docker network create "$NET_NAME"

# 先清掉旧的
for i in 1 2 3; do
  cname="vuav$i"
  echo "[INFO] 停止并删除已有容器: $cname (如果存在)"
  docker rm -f "$cname" 2>/dev/null || true
done

run_vuav() {
  idx="$1"
  name="vuav$idx"

  echo "[INFO] 启动容器 $name (挂到网络 $NET_NAME)"

  docker run -d \
    --name "$name" \
    --net "$NET_NAME" \
    -w /px4/PX4-Autopilot/build/px4_sitl_default \
    "$IMAGE" \
    bash -c "./bin/px4 -s etc/init.d-posix/rcS & sleep 5 && python3 /px4/vuav_agent.py"
}

run_vuav 1
run_vuav 2
run_vuav 3

echo
echo "[INFO] 当前 vuav* 容器状态："
docker ps --filter name=vuav
