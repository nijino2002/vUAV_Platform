#!/usr/bin/env python3
import json
import socket
import time

VUS = {
    "vuav1": ("127.0.0.1", 16001),
    "vuav2": ("127.0.0.1", 16002),
    "vuav3": ("127.0.0.1", 16003),
}


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

    # 7. 再做一轮“高度曲线监控”，更聚焦看高度变化
    monitor_altitude_curve(ticks=10, interval=1.0)

    # 后续如果你想试一下速度指令，可以在这里手动加一句，例如：
    # broadcast_velocity_ned(vx=1.0, vy=0.0, vz=0.0, yaw_rate=0.0)

