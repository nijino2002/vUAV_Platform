#!/usr/bin/env python3
import json
import socket
import time
import os
from pathlib import Path

# ===================== vUAV 集群配置加载（松耦合） =====================

DEFAULT_CONFIG_PATH = "vuav_cluster_config.json"


def load_vus(config_path=None):
    """
    优先从 JSON 配置文件加载 vUAV 列表；
    如果没有配置文件，则退回到当前默认：只连 vuav1 -> 127.0.0.1:6000
    """
    path = config_path or os.environ.get("VUAV_CONFIG", DEFAULT_CONFIG_PATH)
    cfg = Path(path)

    if not cfg.exists():
        print(f"[INFO] config file {cfg} not found, use built-in VUS")
        vus = {
            "vuav1": ("127.0.0.1", 6000),
        }
        for name, addr in vus.items():
            print(f"       {name}: {addr[0]}:{addr[1]}")
        return vus

    print(f"[INFO] loading VUS from {cfg}")
    with cfg.open("r", encoding="utf-8") as f:
        data = json.load(f)

    vus = {}
    for name, item in data.items():
        host = item.get("host")
        port = int(item.get("port", 6000))
        if not host:
            raise ValueError(f"invalid config for {name}: missing 'host'")
        vus[name] = (host, port)

    for name, addr in vus.items():
        print(f"       {name}: {addr[0]}:{addr[1]}")
    return vus


# 现在 VUS 不再硬编码，而是“可配置”
VUS = load_vus()

# ===================== 原来的逻辑保持不变 =====================


def send_cmd(addr, cmd, timeout=3.0):
    s = socket.create_connection(addr, timeout=timeout)
    s.sendall((json.dumps(cmd) + "\n").encode())
    data = s.recv(4096)
    s.close()
    # 这里假定 vuav_agent 一定返回合法 JSON（一旦出错会返回 {"ok": False, ...}）
    return json.loads(data.decode("utf-8"))


def call(name, cmd):
    addr = VUS[name]
    try:
        resp = send_cmd(addr, cmd)
        print(f"[{name}] cmd={cmd} resp={resp}")
        return resp
    except Exception as e:
        print(f"[{name}] ERROR sending {cmd}: {e}")
        return {"ok": False, "error": str(e)}


def broadcast(cmd):
    print(f"\n=== broadcast {cmd} ===")
    results = {}
    for name in VUS:
        results[name] = call(name, cmd)
    return results


def poll_status_once():
    print("\n=== swarm status tick ===")
    for name in VUS:
        call(name, {"cmd": "get_status"})


def broadcast_takeoff(alt=5.0):
    """给所有 vUAV 发起飞命令（通过各自的 vuav_agent）"""
    print(f"\n=== broadcast takeoff alt={alt} ===")
    for name in VUS:
        resp = call(name, {"cmd": "takeoff", "alt": alt})
        # resp 里我们之前在 vuav_agent 里返回了 detail.result 等
        if resp.get("ok"):
            print(f"[{name}] takeoff accepted: {resp}")
        else:
            print(f"[{name}] takeoff FAILED: {resp}")


def monitor_local_position(ticks=10, interval=1.0):
    """循环读取 LOCAL_POSITION_NED，看“飞没飞起来”（全状态）"""
    for i in range(ticks):
        print(f"\n=== local_position tick {i+1}/{ticks} ===")
        for name in VUS:
            resp = call(name, {"cmd": "get_local_position"})
            if resp.get("ok") and resp.get("type") == "local_position":
                x = resp["x"]
                y = resp["y"]
                z = resp["z"]
                vx = resp["vx"]
                vy = resp["vy"]
                vz = resp["vz"]
                print(
                    f"[{name}] pos: x={x:.2f}, y={y:.2f}, z={z:.2f}, "
                    f"vel: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}"
                )
            else:
                print(f"[{name}] get_local_position FAILED: {resp}")
        time.sleep(interval)


def monitor_altitude_curve(ticks=10, interval=1.0):
    """
    高度曲线监控：只关心高度（alt = -z），用来更直观地看“飞没飞起来”。

    返回 alt_history: { "vuav1": [alt_t0, alt_t1, ...], ... }
    """
    alt_history = {name: [] for name in VUS}

    for i in range(ticks):
        print(f"\n=== altitude tick {i+1}/{ticks} ===")
        for name in VUS:
            resp = call(name, {"cmd": "get_local_position"})
            if resp.get("ok") and resp.get("type") == "local_position":
                z = resp["z"]
                alt = -z  # NED: z 向下为正，这里取反当做“向上高度”
                alt_history[name].append(alt)
                print(f"[{name}] alt≈{alt:.2f} m (z={z:.2f})")
            else:
                print(f"[{name}] get_local_position FAILED: {resp}")
        time.sleep(interval)

    print("\n=== altitude summary ===")
    for name, hist in alt_history.items():
        if hist:
            print(f"[{name}] alt min={min(hist):.2f} m, max={max(hist):.2f} m")
        else:
            print(f"[{name}] no altitude data")

    return alt_history


def broadcast_velocity_ned(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
    """
    通过 vuav_agent 的 set_velocity_ned 给机群下发统一速度指令（占位/实验用）。

    注意：要真正生效，一般需要 PX4 在 OFFBOARD 或相应模式下，
    这一层只负责把速度指令打到 MAVLink。
    """
    cmd = {
        "cmd": "set_velocity_ned",
        "vx": float(vx),
        "vy": float(vy),
        "vz": float(vz),
        "yaw_rate": float(yaw_rate),
    }
    print(f"\n=== broadcast velocity NED {cmd} ===")
    for name in VUS:
        call(name, cmd)


def experiment_velocity_forward(duration=5.0, interval=0.5, vx=0.5):
    """
    一个简单的“向前推一推”的速度实验：
      - 每隔 interval 秒对全体 vUAV 广播一次 set_velocity_ned(vx, 0, 0, 0)
      - 同时读取并打印当前位置，便于观察效果
    """
    steps = int(duration / interval)
    print(
        f"\n=== velocity forward experiment: "
        f"duration={duration}s, interval={interval}s, vx={vx} m/s ==="
    )

    for i in range(steps):
        print(f"\n--- velocity step {i+1}/{steps} ---")
        # 1) 下发速度指令
        broadcast_velocity_ned(vx=vx, vy=0.0, vz=0.0, yaw_rate=0.0)

        # 2) 读取一次当前位置
        for name in VUS:
            resp = call(name, {"cmd": "get_local_position"})
            if resp.get("ok") and resp.get("type") == "local_position":
                x = resp["x"]
                y = resp["y"]
                z = resp["z"]
                print(f"[{name}] pos(after vel cmd): x={x:.2f}, y={y:.2f}, z={z:.2f}")
            else:
                print(f"[{name}] get_local_position FAILED: {resp}")

        time.sleep(interval)


if __name__ == "__main__":
    # 1. ping 所有 vUAV，验证 agent 通不通
    broadcast({"cmd": "ping"})

    # 2. 看一轮状态（ATTITUDE / 基本状态）
    poll_status_once()

    # 3. 尝试广播 arm（现在已经走 COMMAND_LONG 了）
    broadcast({"cmd": "arm"})

    # 4. 等 2 秒，让 PX4 有时间处理
    time.sleep(2)
    poll_status_once()

    # 5. 广播 takeoff 到 5m
    broadcast_takeoff(alt=5.0)

    # 6. 起飞后 10 秒内，每秒看一次 LOCAL_POSITION_NED（全状态）
    monitor_local_position(ticks=10, interval=1.0)

    # 7. 做一轮“高度曲线监控”，更聚焦看高度变化
    monitor_altitude_curve(ticks=10, interval=1.0)

    # 8. 简单的“向前速度”实验
    experiment_velocity_forward(duration=5.0, interval=0.5, vx=0.5)
