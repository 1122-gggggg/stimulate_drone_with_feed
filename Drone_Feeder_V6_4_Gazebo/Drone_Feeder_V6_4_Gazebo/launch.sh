#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GZ_SIM_RESOURCE_PATH="$ROOT/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
exec gz sim "$ROOT/worlds/feeder_demo.sdf" "$@"
