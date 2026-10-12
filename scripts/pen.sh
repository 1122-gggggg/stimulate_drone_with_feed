#!/bin/bash
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
PICK=${1:-p50}
export FEEDER=${FEEDER:-1}
export FEED=${FEED:-5.0}
export WF="--era5 $ROOT/wind_data/penghu_era5.json --pick $PICK ${3:-}"
bash "$(dirname "$0")/run_sim.sh" 0 0 0 ${ALT:-20} ${HOLD:-90}
