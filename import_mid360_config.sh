#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_CONFIG="${1:-${HOME}/livox-ws/src/livox_ros_driver2/config/MID360_config.json}"
TARGET_CONFIG="${SCRIPT_DIR}/config/MID360_config.json"

if [ ! -f "${SOURCE_CONFIG}" ]; then
  echo "ERROR: source config not found: ${SOURCE_CONFIG}" >&2
  echo "Usage: $0 /path/to/old/MID360_config.json" >&2
  exit 1
fi

cp "${SOURCE_CONFIG}" "${TARGET_CONFIG}"

echo "Imported MID360 config"
echo "  from: ${SOURCE_CONFIG}"
echo "  to:   ${TARGET_CONFIG}"
echo
echo "Current network fields:"
python3 - <<'PY' "${TARGET_CONFIG}"
import json
import sys

path = sys.argv[1]
with open(path, "r", encoding="utf-8") as f:
    cfg = json.load(f)

host = cfg.get("MID360", {}).get("host_net_info", {})
lidars = cfg.get("lidar_configs", [])
print("  host cmd_data_ip:   ", host.get("cmd_data_ip"))
print("  host point_data_ip: ", host.get("point_data_ip"))
print("  host imu_data_ip:   ", host.get("imu_data_ip"))
for i, lidar in enumerate(lidars):
    print(f"  lidar[{i}] ip:       ", lidar.get("ip"))
PY
