#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
repo="${FASTLIO_SOURCE:-src/FAST_LIO_ROS2}"
patch="$PWD/scripts/fastlio_runtime.patch"
expected=2fffc570a25d0df172720bac034fbdb6a13d2162
if [ "$(git -C "$repo" rev-parse HEAD)" != "$expected" ]; then
  echo "Unsupported FAST-LIO revision; refusing to patch" >&2
  exit 1
fi
if git -C "$repo" apply --reverse --check "$patch" 2>/dev/null; then
  echo "FAST-LIO runtime patch already applied"
else
  git -C "$repo" apply --check "$patch"
  git -C "$repo" apply "$patch"
fi
