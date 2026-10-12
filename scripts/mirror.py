"""Mirror a MAVLink vehicle into the Gazebo twin world (read-only: the drone is never commanded).

  bash scripts/twin.sh                                  # start the twin world (no SITL)
  python3 scripts/mirror.py --conn udpin:0.0.0.0:14550  # telemetry forwarded from Mission Planner / MAVProxy
  python3 scripts/mirror.py --conn /dev/ttyUSB0,57600   # telemetry radio / USB attached to this Linux

Reads LOCAL_POSITION_NED + ATTITUDE (NED / FRD, origin = the drone's EKF origin) and sets the pose of
model `bigquad_twin` (ENU / FLU) through gz-sim's /world/<w>/set_pose service.
Between telemetry packets the position is extrapolated with the reported velocity (max 0.5 s).
"""
import argparse
import json
import math
import os
import time

import numpy as np
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node
from pymavlink import mavutil

R_ENU_NED = np.array([[0.0, 1, 0], [1, 0, 0], [0, 0, -1]])
R_FRD_FLU = np.diag([1.0, -1, -1])


def rot_ned_frd(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def quat_xyzw(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 2 * math.sqrt(t + 1)
        return (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        return s / 4, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s
    if R[1, 1] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        return (R[0, 1] + R[1, 0]) / s, s / 4, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s
    s = 2 * math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
    return (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4, (R[1, 0] - R[0, 1]) / s


def main():
    cfg = {"world": "bigquad_twin", "model": "bigquad_twin", "rest_z": 0.64}
    try:
        cfg.update(json.load(open(os.path.expanduser("~/simulators/bigquad/twin.json"))))
    except FileNotFoundError:
        pass
    ap = argparse.ArgumentParser()
    ap.add_argument("--conn", default="udpin:0.0.0.0:14550")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--world", default=cfg["world"])
    ap.add_argument("--model", default=cfg["model"])
    ap.add_argument("--rest-z", type=float, default=cfg["rest_z"], help="model origin height above ground when landed (m)")
    ap.add_argument("--offset-n", type=float, default=0.0, help="where the drone's origin sits in the world, north (m)")
    ap.add_argument("--offset-e", type=float, default=0.0, help="... east (m)")
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--request-rates", action="store_true",
                    help="ask the autopilot for 20 Hz ATTITUDE / LOCAL_POSITION_NED (sends SET_MESSAGE_INTERVAL; off = purely passive)")
    a = ap.parse_args()

    m = mavutil.mavlink_connection(a.conn, baud=a.baud)
    print("waiting for heartbeat on", a.conn)
    m.wait_heartbeat()
    print("heartbeat from system", m.target_system)
    if a.request_rates:
        for mid in (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED):
            m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                    0, mid, 50000, 0, 0, 0, 0, 0)

    node = Node()
    service = f"/world/{a.world}/set_pose"
    req = Pose()
    req.name = a.model

    pos = vel = None  # ENU, metres / m/s
    quat = (0.0, 0.0, 0.0, 1.0)
    t_pos = t_rx = time.time()
    att = None
    period = 1.0 / a.hz
    next_send = time.time()
    fails = 0
    lost = False
    while True:
        wait = max(0.0, next_send - time.time())
        msg = m.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=True, timeout=wait or 0.001)
        while msg is not None:
            t_rx = time.time()
            if msg.get_type() == "LOCAL_POSITION_NED":
                pos = np.array([msg.y + a.offset_e, msg.x + a.offset_n, -msg.z + a.rest_z])
                vel = np.array([msg.vy, msg.vx, -msg.vz])
                t_pos = t_rx
            else:
                R = R_ENU_NED @ rot_ned_frd(msg.roll, msg.pitch, msg.yaw) @ R_FRD_FLU
                quat = quat_xyzw(R)
            msg = m.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
        now = time.time()
        if now < next_send:
            continue
        next_send = now + period
        if pos is None:
            continue
        if now - t_rx > 2.0:
            if not lost:
                print("telemetry lost, holding last pose")
            lost = True
        elif lost:
            print("telemetry back")
            lost = False
        p = pos + vel * min(now - t_pos, 0.5)
        req.position.x, req.position.y, req.position.z = p
        req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = quat
        ok, rep = node.request(service, req, Pose, Boolean, 200)
        if not (ok and rep.data):
            fails += 1
            if fails % 30 == 1:
                print(f"set_pose failed ({service}); is the twin world running? GZ_IP=127.0.0.1 exported?")
        else:
            fails = 0


if __name__ == "__main__":
    main()
