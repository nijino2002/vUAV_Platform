#!/usr/bin/env python3
import json
import socket
import threading
from pymavlink import mavutil

MAVLINK_URL = "udp:127.0.0.1:14540"
AGENT_LISTEN_ADDR = ("0.0.0.0", 6000)
RAD2DEG = 57.29577951308232


def mavlink_connect():
    print(f"[agent] connecting to PX4 via {MAVLINK_URL} ...")
    m = mavutil.mavlink_connection(MAVLINK_URL)
    m.wait_heartbeat(timeout=10)
    print(f"[agent] HEARTBEAT from PX4: sysid={m.target_system}, compid={m.target_component}")
    return m


def send_command_long(mav, command, params, timeout=2.0):
    """
    发送 MAV_CMD_*，并等待 COMMAND_ACK。
    params: 长度为 7 的 list/tuple -> param1..param7
    """
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        command,
        0,
        *params
    )
    ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
    if not ack:
        return {"ok": False, "error": "no COMMAND_ACK"}

    ok = (ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED)
    return {
        "ok": ok,
        "result": int(ack.result),
        "command": int(ack.command),
    }


def process_request(req, mav):
    """处理来自 swarm_controller 的单条请求。"""
    cmd = req.get("cmd")

    # --- 基础命令 ---
    if cmd == "ping":
        return {"ok": True, "reply": "pong"}

    if cmd == "get_attitude":
        msg = mav.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
        if msg is None:
            return {"ok": False, "error": "no ATTITUDE"}

        return {
            "ok": True,
            "type": "attitude",
            "roll_deg":  msg.roll  * RAD2DEG,
            "pitch_deg": msg.pitch * RAD2DEG,
            "yaw_deg":   msg.yaw   * RAD2DEG,
        }

    if cmd == "get_status":
        msg = mav.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
        if msg is None:
            return {"ok": False, "error": "no ATTITUDE"}

        return {
            "ok": True,
            "type": "status",
            "attitude": {
                "roll_deg":  msg.roll  * RAD2DEG,
                "pitch_deg": msg.pitch * RAD2DEG,
                "yaw_deg":   msg.yaw   * RAD2DEG,
            },
        }

    # --- 控制命令：arm / disarm ---
    if cmd == "arm":
        resp = send_command_long(
            mav,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1, 0, 0, 0, 0, 0, 0],  # param1 = 1 -> arm
        )
        return {"ok": resp["ok"], "detail": resp}

    if cmd == "disarm":
        resp = send_command_long(
            mav,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [0, 0, 0, 0, 0, 0, 0],  # param1 = 0 -> disarm
        )
        return {"ok": resp["ok"], "detail": resp}

    # --- 控制命令：简单 takeoff ---
    if cmd == "takeoff":
        alt = float(req.get("alt", 5.0))  # 目标高度，单位 m

        resp = send_command_long(
            mav,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            [0, 0, 0, 0, 0, 0, alt],
        )
        return {"ok": resp["ok"], "detail": resp}

    # 还没实现的命令，统一返回 unknown
    return {"ok": False, "error": f"unknown cmd {cmd}"}


def handle_client(conn, addr, mav):
    print(f"[agent] client connected from {addr}")
    with conn:
        buf = b""
        while True:
            data = conn.recv(4096)
            if not data:
                break
            buf += data
            # 一行一个 JSON，用换行符分包
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    req = json.loads(line.decode("utf-8"))
                except Exception as e:
                    print(f"[agent] bad json from {addr}: {e}")
                    continue
                resp = process_request(req, mav)
                conn.sendall((json.dumps(resp) + "\n").encode("utf-8"))
    print(f"[agent] client disconnected from {addr}")


def agent_main():
    mav = mavlink_connect()

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(AGENT_LISTEN_ADDR)
    s.listen(5)
    print(f"[agent] listening on {AGENT_LISTEN_ADDR[0]}:{AGENT_LISTEN_ADDR[1]}")

    try:
        while True:
            conn, addr = s.accept()
            th = threading.Thread(
                target=handle_client,
                args=(conn, addr, mav),
                daemon=True
            )
            th.start()
    finally:
        s.close()


if __name__ == "__main__":
    agent_main()

