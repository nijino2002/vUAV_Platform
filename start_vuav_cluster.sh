#!/usr/bin/env bash
set -e

# 使用的镜像名
IMAGE="px4-vuav-agent:v4-20251127"

echo "[INFO] 使用镜像: $IMAGE"

# 先把已有的 vuav1/2/3 删掉，避免名字/端口冲突
for i in 1 2 3; do
  cname="vuav$i"
  echo "[INFO] 停止并删除已有容器: $cname (如果存在)"
  docker rm -f "$cname" 2>/dev/null || true
done

run_vuav() {
  idx="$1"
  port="$2"
  name="vuav$idx"

  echo "[INFO] 启动容器 $name (宿主机端口 $port → 容器 6000/tcp)"

  docker run -d \
    --name "$name" \
    -w /px4/PX4-Autopilot/build/px4_sitl_default \
    -p "${port}:6000/tcp" \
    "$IMAGE" \
    bash -c "./bin/px4 -s etc/init.d-posix/rcS & sleep 5 && python3 /px4/vuav_agent.py"
}

# 启动三架 vUAV
run_vuav 1 16001
run_vuav 2 16002
run_vuav 3 16003

echo
echo "[INFO] 当前 vuav* 容器状态："
docker ps --filter name=vuav

