#!/usr/bin/env python3
import json
import socket
import threading
import traceback
import time

from pymavlink import mavutil

MAVLINK_URL = "udp:127.0.0.1:14540"
AGENT_LISTEN_ADDR = ("0.0.0.0", 6000)
RAD2DEG = 57.29577951308232

# ---- 全局状态缓存（由 MAVLink 读线程维护） ----
STATE = {
    "ATTITUDE": None,
    "LOCAL_POSITION_NED": None,
    "_ts_ATTITUDE": 0.0,
    "_ts_LOCAL_POSITION_NED": 0.0,
}
STATE_LOCK = threading.Lock()


def mavlink_connect():
    print(f"[agent] connecting to PX4 via {MAVLINK_URL} ...", flush=True)
    m = mavutil.mavlink_connection(MAVLINK_URL)
    m.wait_heartbeat(timeout=10)
    print(
        f"[agent] HEARTBEAT from PX4: sysid={m.target_system}, "
        f"compid={m.target_component}",
        flush=True,
    )
    return m


def mavlink_reader(mav):
    """
    后台线程：不停从 PX4 读 MAVLink 消息，把最新的
    ATTITUDE / LOCAL_POSITION_NED 写入 STATE。
    """
    print("[agent] mavlink_reader started", flush=True)
    while True:
        try:
            msg = mav.recv_match(blocking=True, timeout=1.0)
        except Exception as e:
            print(f"[agent] mavlink_reader exception: {e}", flush=True)
            continue

        if msg is None:
            continue

        mtype = msg.get_type()
        now = time.time()

        if mtype in ("ATTITUDE", "LOCAL_POSITION_NED"):
            with STATE_LOCK:
                STATE[mtype] = msg
                STATE[f"_ts_{mtype}"] = now


def send_command_long(mav, command, params, timeout=2.0):
    """发送 MAV_CMD_* 并等待 COMMAND_ACK"""
    print(f"[agent] send_command_long: cmd={command}, params={params}", flush=True)
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        command,
        0,
        *params,
    )
    ack = mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout)
    if not ack:
        print("[agent] send_command_long: no COMMAND_ACK", flush=True)
        return {"ok": False, "error": "no COMMAND_ACK"}

    ok = ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    print(
        f"[agent] send_command_long: ack result={ack.result}, "
        f"command={ack.command}",
        flush=True,
    )
    return {
        "ok": ok,
        "result": int(ack.result),
        "command": int(ack.command),
    }


def _get_cached_msg(msg_type: str, max_age: float = 2.0):
    """
    从 STATE 中取最新的指定类型消息，并做简单“过期”检查。
    max_age: 允许的最大时间（秒），超过视为无效。
    """
    now = time.time()
    with STATE_LOCK:
        msg = STATE.get(msg_type)
        ts = STATE.get(f"_ts_{msg_type}", 0.0)

    if msg is None:
        return None, {"ok": False, "error": f"no {msg_type} cached"}

    age = now - ts
    if age > max_age:
        return None, {"ok": False, "error": f"{msg_type} too old", "age": age}

    return msg, None


def process_request(req, mav):
    """处理来自 swarm_controller 的单条请求。"""
    cmd = req.get("cmd")
    print(f"[agent] process_request cmd={cmd}, payload={req}", flush=True)

    # --- 基础命令 ---
    if cmd == "ping":
        return {"ok": True, "reply": "pong"}

    if cmd == "get_attitude":
        msg, err = _get_cached_msg("ATTITUDE")
        if msg is None:
            return err
        return {
            "ok": True,
            "type": "attitude",
            "roll_deg": msg.roll * RAD2DEG,
            "pitch_deg": msg.pitch * RAD2DEG,
            "yaw_deg": msg.yaw * RAD2DEG,
        }

    if cmd == "get_local_position":
        msg, err = _get_cached_msg("LOCAL_POSITION_NED")
        if msg is None:
            return err
        return {
            "ok": True,
            "type": "local_position",
            "x": float(msg.x),
            "y": float(msg.y),
            "z": float(msg.z),
            "vx": float(msg.vx),
            "vy": float(msg.vy),
            "vz": float(msg.vz),
        }

    if cmd == "get_status":
        msg, err = _get_cached_msg("ATTITUDE")
        if msg is None:
            return err
        return {
            "ok": True,
            "type": "status",
            "attitude": {
                "roll_deg": msg.roll * RAD2DEG,
                "pitch_deg": msg.pitch * RAD2DEG,
                "yaw_deg": msg.yaw * RAD2DEG,
            },
        }

    # --- 控制命令：arm / disarm ---
    if cmd == "arm":
        resp = send_command_long(
            mav,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1, 0, 0, 0, 0, 0, 0],
        )
        return {"ok": resp["ok"], "detail": resp}

    if cmd == "disarm":
        resp = send_command_long(
            mav,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [0, 0, 0, 0, 0, 0, 0],
        )
        return {"ok": resp["ok"], "detail": resp}

    # --- 控制命令：简单 takeoff ---
    if cmd == "takeoff":
        alt = float(req.get("alt", 5.0))
        resp = send_command_long(
            mav,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            [0, 0, 0, 0, 0, 0, alt],
        )
        return {"ok": resp["ok"], "detail": resp}

    # --- 控制命令：简单速度控制（本地 NED） ---
    if cmd == "set_velocity_ned":
        vx = float(req.get("vx", 0.0))
        vy = float(req.get("vy", 0.0))
        vz = float(req.get("vz", 0.0))
        yaw_rate = float(req.get("yaw_rate", 0.0))

        # 只用速度 + yaw_rate
        type_mask = 1479
        time_boot_ms = int(mav.time_since("SYSTEM_BOOT") * 1000)

        print(
            f"[agent] set_velocity_ned: vx={vx}, vy={vy}, vz={vz}, "
            f"yaw_rate={yaw_rate}",
            flush=True,
        )

        mav.mav.set_position_target_local_ned_send(
            time_boot_ms,
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0.0,
            0.0,
            0.0,
            vx,
            vy,
            vz,
            0.0,
            0.0,
            0.0,
            0.0,
            yaw_rate,
        )

        return {
            "ok": True,
            "type": "set_velocity_ned",
            "vx": vx,
            "vy": vy,
            "vz": vz,
            "yaw_rate": yaw_rate,
        }

    # --- 控制命令：简单航点占位 ---
    if cmd == "goto_local_ned":
        x = float(req.get("x", 0.0))
        y = float(req.get("y", 0.0))
        z = float(req.get("z", -5.0))
        return {
            "ok": False,
            "error": "goto_local_ned not implemented yet",
            "target": {"x": x, "y": y, "z": z},
        }

    # 未实现命令
    return {"ok": False, "error": f"unknown cmd {cmd}"}


def handle_client(conn, addr, mav):
    print(f"[agent] client connected from {addr}", flush=True)
    # 给这个连接设置一个整体超时，防止一直挂住
    conn.settimeout(10.0)
    with conn:
        buf = b""
        while True:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                print(f"[agent] recv timeout from {addr}, closing", flush=True)
                break
            except Exception as e:
                print(f"[agent] recv error from {addr}: {e}", flush=True)
                break

            if not data:
                print(f"[agent] connection closed by peer {addr}", flush=True)
                break

            buf += data
            print(f"[agent] raw data from {addr}: {buf!r}", flush=True)

            # 按行拆包
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    text = line.decode("utf-8")
                    print(f"[agent] line from {addr}: {text!r}", flush=True)
                    req = json.loads(text)
                except Exception as e:
                    print(f"[agent] bad json from {addr}: {e}", flush=True)
                    resp = {"ok": False, "error": f"bad json: {e}"}
                else:
                    try:
                        resp = process_request(req, mav)
                    except Exception as e:
                        print(f"[agent] handler exception for {addr}: {e}", flush=True)
                        traceback.print_exc()
                        resp = {"ok": False, "error": f"handler exception: {e}"}

                try:
                    payload = json.dumps(resp) + "\n"
                    conn.sendall(payload.encode("utf-8"))
                    print(
                        f"[agent] send resp to {addr}: {payload!r}",
                        flush=True,
                    )
                except Exception as e:
                    print(f"[agent] send error to {addr}: {e}", flush=True)
                    return
    print(f"[agent] client disconnected from {addr}", flush=True)


def agent_main():
    mav = mavlink_connect()

    # 启动后台 MAVLink 读线程
    reader = threading.Thread(target=mavlink_reader, args=(mav,), daemon=True)
    reader.start()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(AGENT_LISTEN_ADDR)
    s.listen(5)
    print(
        f"[agent] listening on {AGENT_LISTEN_ADDR[0]}:{AGENT_LISTEN_ADDR[1]}",
        flush=True,
    )

    try:
        while True:
            conn, addr = s.accept()
            th = threading.Thread(
                target=handle_client,
                args=(conn, addr, mav),
                daemon=True,
            )
            th.start()
    finally:
        s.close()


if __name__ == "__main__":
    agent_main()
