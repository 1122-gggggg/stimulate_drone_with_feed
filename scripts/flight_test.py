import math
import statistics as st
import sys
import time

from pymavlink import mavutil

alt_target = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
hold = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0

m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
m.wait_heartbeat(timeout=30)
ts, tc = m.target_system, m.target_component
m.mav.request_data_stream_send(ts, tc, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)


def cmd(c, *p):
    m.mav.command_long_send(ts, tc, c, 0, *(list(p) + [0] * (7 - len(p))))


def set_mode(name):
    m.mav.set_mode_send(ts, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, m.mode_mapping()[name])


t0 = time.time()
while time.time() - t0 < 90:
    s = m.recv_match(type="STATUSTEXT", blocking=True, timeout=2)
    if s and "is using GPS" in s.text:
        break
time.sleep(3)
set_mode("GUIDED")
armed = False
t0 = time.time()
while time.time() - t0 < 120 and not armed:
    cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196)
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    armed = bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    time.sleep(1)
print("armed", armed, "after", round(time.time() - t0), "s")
if not armed:
    for _ in range(10):
        s = m.recv_match(type="STATUSTEXT", blocking=True, timeout=2)
        if s:
            print(s.text)
    sys.exit(1)

cmd(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt_target)
t_start = time.time()
rolls, pitches, thr, alts, gs = [], [], [], [], []
reached = None
log = []
speed = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
last_sp = 0.0
while time.time() - t_start < 90 + hold:
    if reached and speed and time.time() - last_sp > 0.2:
        m.mav.set_position_target_local_ned_send(
            0, ts, tc, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
            0, 0, 0, speed, 0, 0, 0, 0, 0, 0, 0)
        last_sp = time.time()
    msg =m.recv_match(type=["ATTITUDE", "VFR_HUD", "LOCAL_POSITION_NED", "SERVO_OUTPUT_RAW", "STATUSTEXT"], blocking=True, timeout=2)
    if not msg:
        continue
    t = msg.get_type()
    if t == "STATUSTEXT":
        print("STATUS:", msg.text)
    elif t == "LOCAL_POSITION_NED":
        alts.append(-msg.z)
        if reached is None and -msg.z >= 0.95 * alt_target:
            reached = time.time()
            print(f"reached {-msg.z:.2f} m in {reached - t_start:.1f} s")
    elif t == "ATTITUDE" and reached:
        rolls.append(math.degrees(msg.roll))
        pitches.append(math.degrees(msg.pitch))
    elif t == "VFR_HUD" and reached:
        thr.append(msg.throttle)
        gs.append(msg.groundspeed)
    elif t == "SERVO_OUTPUT_RAW" and reached:
        log.append((msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw))
    if reached and time.time() - reached > hold:
        break

if reached and rolls:
    print(f"hold {hold}s: alt mean {st.mean(alts[-100:]):.2f} sd {st.pstdev(alts[-100:]):.2f}")
    print(f"roll mean {st.mean(rolls):.2f} sd {st.pstdev(rolls):.2f} max|{max(map(abs, rolls)):.1f}|")
    print(f"pitch mean {st.mean(pitches):.2f} sd {st.pstdev(pitches):.2f} max|{max(map(abs, pitches)):.1f}|")
    print(f"throttle% mean {st.mean(thr):.1f}  groundspeed mean {st.mean(gs):.2f} max {max(gs):.2f}  last10 {st.mean(gs[-50:]):.2f}")
    print(f"alt min {min(alts[-150:]):.2f} max {max(alts[-150:]):.2f}")
    if log:
        print("servo mean", [round(st.mean(c)) for c in zip(*log)])
else:
    print("did not reach target altitude; max alt", max(alts) if alts else None)
set_mode("LAND")
