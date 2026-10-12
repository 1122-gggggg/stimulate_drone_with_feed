#!/bin/bash
export PATH=$HOME/.local/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin
set -e
cd ~/simulators
[ -d ardupilot_gazebo ] || git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git 2>&1 | tail -1
pip3 install --user -q empy==3.3.4 pexpect future pymavlink MAVProxy pyserial 2>&1 | tail -2
cd ardupilot
./waf configure --board sitl 2>&1 | tail -3
./waf copter 2>&1 | tail -4
cd ~/simulators/ardupilot_gazebo
export GZ_VERSION=harmonic
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo 2>&1 | tail -3
make -j12 2>&1 | tail -4
ls ~/simulators/ardupilot_gazebo/build/*.so
