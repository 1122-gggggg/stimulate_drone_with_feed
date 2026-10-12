#!/bin/bash
# SITL + Gazebo with the feeder, the site obstacles and the Xiyu wind:
#   mean wind / gust from the NCDR station snapshot, spatial structure from the OpenFOAM field.
#   GUI=1 bash scripts/xiyu.sh [max_1h|max_3h|max_6h|max_12h|max_24h|max_48h]
# Then fly it from Python (scripts/control.py, scripts/feed_demo.py).
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
WIN=${1:-max_48h}
export SITE=$ROOT/wind_cfd/site.json
export FEEDER=${FEEDER:-1} FEED=${FEED:-5.0}
export WF="--ncdr $ROOT/wind_data/xiyu_ncdr.json --window $WIN --field $ROOT/wind_data/xiyu_d022.npz"
exec bash "$(dirname "$0")/serve.sh" 0 0 0 ${ALT:-20} ${HOLD:-600}
