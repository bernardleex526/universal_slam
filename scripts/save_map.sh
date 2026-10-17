#!/usr/bin/env bash
set -euo pipefail
exec python3 "$(dirname "$0")/../slam_nav_core/scripts/save_map.py" "$@"
