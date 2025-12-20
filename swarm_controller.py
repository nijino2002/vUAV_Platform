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

# ===================== 超时策略（关键：修复 offboard_takeoff 3s 超时） =====================

DEFAULT_RPC_TIMEOUT = 3.0

# 基础超时表：长命令给更长
CMD_TIMEOUT_BASE = {
    "offboard_takeoff": 30.0,  # 兜底，实际还会按 hold_time 拉长
    "takeoff": 15.0,
    "land": 15.0,
    "rtl": 15.0,
}

def pick_timeout(cmd: dict) -> float:
    """
    根据 cmd 类型选择合适超时：
    - offboard_takeoff：必须 >= hold_time，否则 controller 一定超时
    """
    c = cmd.get("cmd", "")
    t = float(CMD_TIMEOUT_BASE.get(c, DEFAULT_RPC_TIMEOUT))

    if c == "offboard_takeoff":
        hold = float(cmd.get("hold_time", 0.0) or 0.0)
        pre = float(cmd.get("pre_offboard_sec", 0.0) or 0.0)
        # 加 10 秒缓冲，避免边界抖动/调度延迟
        t = max(t, hold + pre + 10.0)

    return float(t)

# ===================== 底层 RPC 封装 =====================

def _recv_until_newline(sock: socket.socket, max_bytes: int = 1024 * 1024) -> bytes:
    """
    vuav_agent 每次响应都是一行 JSON + '\n'。
    用 recv(4096) 可能拿到半包，所以这里按行读到 '\n' 为止。
    """
    buf = b""
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            break
        buf += chunk
        if b"\n" in buf:
            line, _rest = buf.split(b"\n", 1)
            return line
        if len(buf) > max_bytes:
            raise RuntimeError("response too large (no newline?)")
    return buf


def send_cmd(addr, cmd, timeout=DEFAULT_RPC_TIMEOUT):
    """
    发送一条 JSON 命令（以 '\n' 结尾），并读取一行 JSON 响应。
    """
    s = socket.create_connection(addr, timeout=timeout)
    try:
        s.settimeout(timeout)
        s.sendall((json.dumps(cmd) + "\n").encode("utf-8"))
        line = _recv_until_newline(s)
        if not line:
            raise RuntimeError("empty response")
        return json.loads(line.decode("utf-8"))
    finally:
        try:
            s.close()
        except Exception:
            pass


def wait_agent_ready(name, timeout=30.0, interval=0.2):
    """
    解决你日志里第一波 Connection refused：controller 抢跑，agent 还没 listen。
    这里会反复尝试 ping，直到 ok 或超时。
    """
    addr = VUS[name]
    deadline = time.time() + timeout
    last_err = None

    while time.time() < deadline:
        try:
            resp = send_cmd(addr, {"cmd": "ping"}, timeout=1.0)
            if resp.get("ok"):
                print(f"[{name}] agent READY at {addr[0]}:{addr[1]}")
                return True
            last_err = f"ping not ok: {resp}"
        except Exception as e:
            last_err = str(e)

        time.sleep(interval)

    raise RuntimeError(f"[{name}] agent NOT ready within {timeout}s, last_err={last_err}")


def wait_all_ready(timeout_each=30.0):
    for name in VUS:
        wait_agent_ready(name, timeout=timeout_each)


def call(name, cmd, retries=3, retry_interval=0.25):
    addr = VUS[name]

    # ——关键修复：每条命令用“合适的 timeout”——
    timeout = pick_timeout(cmd)

    # 长命令（offboard_takeoff）不要乱重试：一次跑完最靠谱
    effective_retries = 0 if cmd.get("cmd") == "offboard_takeoff" else retries

    last_err = None
    for i in range(effective_retries + 1):
        try:
            resp = send_cmd(addr, cmd, timeout=timeout)
            print(f"[{name}] cmd={cmd} resp={resp}")
            return resp
        except Exception as e:
            last_err = e
            # 只对“连接未就绪/瞬态失败”做短重试
            msg = str(e)
            if "Connection refused" in msg or "Errno 111" in msg or "timed out" in msg:
                if i < effective_retries:
                    time.sleep(retry_interval)
                    continue
            print(f"[{name}] ERROR sending {cmd}: {e}")
            return {"ok": False, "error": str(e)}

    return {"ok": False, "error": str(last_err) if last_err else "unknown error"}


# ===================== 群体操作与实验函数 =====================

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
    """给所有 vUAV 发起飞命令（通过各自的 vuav_agent，传统 NAV_TAKEOFF 流程）"""
    print(f"\n=== broadcast takeoff alt={alt} (NAV_TAKEOFF) ===")
    for name in VUS:
        resp = call(name, {"cmd": "takeoff", "alt": alt})
        if resp.get("ok"):
            print(f"[{name}] takeoff accepted: {resp}")
        else:
            print(f"[{name}] takeoff FAILED: {resp}")


def broadcast_offboard_takeoff(alt=5.0, climb_rate=1.0, hold_time=None):
    """
    基于 agent 的 offboard_takeoff 序列进行广播：
      - 在 agent 侧完成 arm + OFFBOARD 模式切换 + 连续 setpoint 拉升
    """
    if hold_time is None:
        hold_time = alt / climb_rate + 3.0

    cmd = {
        "cmd": "offboard_takeoff",
        "alt": float(alt),
        "climb_rate": float(climb_rate),
        "pre_offboard_sec": 1.0,
        "hold_time": float(hold_time),
    }

    print(f"\n=== broadcast OFFBOARD takeoff {cmd} ===")
    results = {}
    for name in VUS:
        results[name] = call(name, cmd)
    return results


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
    # 0) 关键修复：先等所有 agent 真正 READY（避免第一波 Connection refused）
    wait_all_ready(timeout_each=30.0)

    # 1) ping 所有 vUAV，验证 agent 通不通
    broadcast({"cmd": "ping"})

    # 2) 看一轮状态（ATTITUDE / 基本状态）
    poll_status_once()

    # 3) 使用 OFFBOARD 起飞序列（包含 arm + OFFBOARD 切换 + 连续速度 setpoint）
    broadcast_offboard_takeoff(alt=5.0, climb_rate=1.0)
    call("vuav1", {"cmd": "get_mode_state"})

    # 4) 起飞后 10 秒内，每秒看一次 LOCAL_POSITION_NED（全状态）
    monitor_local_position(ticks=10, interval=1.0)

    # 5) 再做一轮“高度曲线监控”，更聚焦看高度变化
    monitor_altitude_curve(ticks=10, interval=1.0)

    # 6) 简单的“向前速度”实验（此时理论上仍处于 OFFBOARD）
    experiment_velocity_forward(duration=5.0, interval=0.5, vx=0.5)
