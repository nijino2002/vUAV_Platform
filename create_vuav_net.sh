#!/usr/bin/env bash
set -e

NET_NAME="vuav-net"

if docker network inspect "$NET_NAME" >/dev/null 2>&1; then
  echo "[INFO] docker network $NET_NAME already exists."
else
  echo "[INFO] creating docker network $NET_NAME ..."
  docker network create "$NET_NAME"
  echo "[INFO] docker network $NET_NAME created."
fi
