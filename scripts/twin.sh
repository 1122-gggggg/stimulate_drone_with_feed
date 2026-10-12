#!/bin/bash
# Digital-twin world for mirroring a real drone (no SITL, no physics for the drone).
#   GUI=1 bash scripts/twin.sh        # then: python3 scripts/mirror.py --conn <telemetry source>
# Same scene as xiyu.sh (feeder visuals + obstacles from wind_cfd/site.json).
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
export PATH=$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin
AG=~/simulators/ardupilot_gazebo
export GZ_SIM_SYSTEM_PLUGIN_PATH=$AG/build
export GZ_SIM_RESOURCE_PATH=$AG/models:$AG/worlds
export GZ_IP=127.0.0.1
pkill -f "gz sim"; pkill -f arducopter; pkill -f sim_vehicle; sleep 2
python3 $ROOT/generator/gen_bigquad.py --feeder --feed ${FEED:-5.0} --site ${SITE:-$ROOT/wind_cfd/site.json} > /dev/null || exit 1
python3 $ROOT/generator/gen_twin.py || exit 1
mkdir -p ~/simulators/logs
if [ -n "$GUI" ]; then
  export DISPLAY=${DISPLAY:-:0}; export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}
  nohup gz sim -r bigquad_twin.sdf > ~/simulators/logs/gz.log 2>&1 &
else
  nohup gz sim -s -r --headless-rendering bigquad_twin.sdf > ~/simulators/logs/gz.log 2>&1 &
fi
sleep 8
echo "twin world up: /world/bigquad_twin/set_pose"
