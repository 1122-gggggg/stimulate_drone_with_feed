#!/bin/bash
ROOT=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
export PATH=$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin
PAYLOAD=${1:-0}
WIND=${2:-0}
WDIR=${3:-0}
ALT=${4:-10}
HOLD=${5:-20}
SPEED=${6:-0}
FT=$(dirname "$0")/flight_test.py
AG=~/simulators/ardupilot_gazebo
export GZ_SIM_SYSTEM_PLUGIN_PATH=$AG/build
export GZ_SIM_RESOURCE_PATH=$AG/models:$AG/worlds
export GZ_IP=127.0.0.1
pkill -f "gz sim"; pkill -f arducopter; pkill -f sim_vehicle; sleep 2
python3 $ROOT/generator/gen_bigquad.py --payload $PAYLOAD --wind $WIND --wind-dir $WDIR --fuel ${FUEL:-3.0} ${WF:+--wind-ext} ${FEEDER:+--feeder --feed ${FEED:-5.0}} ${SITE:+--site $SITE}
mkdir -p ~/simulators/logs
nohup gz sim -s -r --headless-rendering bigquad_runway.sdf > ~/simulators/logs/gz.log 2>&1 &
sleep 12
cd ~/simulators/ardupilot/ArduCopter
nohup python3 ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --no-mavproxy --no-rebuild -w \
  --add-param-file ~/simulators/bigquad/bigquad.parm > ~/simulators/logs/sitl.log 2>&1 &
sleep 20
grep -i -E "error|fail" ~/simulators/logs/gz.log | grep -v -i -E "GstCamera|Reason|Requested|Resolved" | head -5
if [ -n "$WF" ]; then
  python3 $ROOT/generator/windfeed.py --alt $ALT --duration $((HOLD+150)) --ramp 20 $WF > ~/simulators/logs/windfeed.log 2>&1 &
  sleep 2; cat ~/simulators/logs/windfeed.log
fi
python3 "$FT" $ALT $HOLD $SPEED
