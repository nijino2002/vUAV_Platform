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

if __name__ == "__main__":
    # 1. 先 ping 一下所有 vUAV
    broadcast({"cmd": "ping"})

    # 2. 看一轮状态
    poll_status_once()

    # 3. 尝试广播 arm（可能会被 PX4 拒绝，但链路打通）
    broadcast({"cmd": "arm"})

    # 4. 等一小会，再看一轮状态
    time.sleep(2)
    poll_status_once()

    # 5. 尝试广播 takeoff 到 5m
    broadcast({"cmd": "takeoff", "alt": 5.0})

    # 后面你可以在这里加循环，每隔 1s poll 一次
    # while True:
    #     poll_status_once()
    #     time.sleep(1.0)

