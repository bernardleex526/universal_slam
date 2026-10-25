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
elif git -C "$repo" apply --check "$patch" 2>/dev/null; then
  git -C "$repo" apply "$patch"
else
  # 源码树上已有旧版补丁或手工改动：先恢复上游再重打 Livox 与 runtime 补丁。
  echo "FAST-LIO tree has an older runtime patch or local edits. Reset and re-patch:" >&2
  echo "  git -C $repo checkout -- . && bash scripts/patch_fastlio_livox.sh && bash scripts/patch_fastlio_runtime.sh" >&2
  exit 1
fi
