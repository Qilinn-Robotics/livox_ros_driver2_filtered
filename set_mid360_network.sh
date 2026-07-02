#!/usr/bin/env bash
set -e

if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <host_ip> <lidar_ip>" >&2
  echo "Example: $0 192.168.1.5 192.168.1.12" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="${SCRIPT_DIR}/config/MID360_config.json"
HOST_IP="$1"
LIDAR_IP="$2"

python3 - <<'PY' "${CONFIG_PATH}" "${HOST_IP}" "${LIDAR_IP}"
import json
import sys

path, host_ip, lidar_ip = sys.argv[1:4]
with open(path, "r", encoding="utf-8") as f:
    cfg = json.load(f)

host = cfg.setdefault("MID360", {}).setdefault("host_net_info", {})
for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
    host[key] = host_ip
if "log_data_ip" in host:
    host["log_data_ip"] = ""

lidars = cfg.setdefault("lidar_configs", [])
if not lidars:
    lidars.append({})
lidars[0]["ip"] = lidar_ip

with open(path, "w", encoding="utf-8") as f:
    json.dump(cfg, f, indent=2)
    f.write("\n")
PY

echo "Updated ${CONFIG_PATH}"
echo "  host_ip=${HOST_IP}"
echo "  lidar_ip=${LIDAR_IP}"
