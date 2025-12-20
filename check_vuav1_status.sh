#!/usr/bin/env bash
set -euo pipefail

cname="vuav1"

docker exec -it "$cname" bash -lc '
  echo "[A] px4:"
  pgrep -a px4 || true
  echo

  echo "[C] agent (python only):"
  pgrep -af "python3 /px4/vuav_agent.py" || true
  echo

  echo "[D] agent port 6000 listening (connect test):"
  python3 - <<'"'"'PY'"'"'
import socket
s = socket.socket()
s.settimeout(1.0)
try:
    s.connect(("127.0.0.1", 6000))
    print("OK: 127.0.0.1:6000 is accepting connections")
except Exception as e:
    print("FAIL:", e)
finally:
    s.close()
PY
'

echo
echo "[B] bind failed (last 10m from docker logs):"
docker logs "$cname" --since 10m | grep -n "bind failed" || echo "(OK)"
