"""Control the SITL drone from Python (pymavlink).

Start SITL first:   GUI=1 bash scripts/serve.sh 0 0 0 20 600
Run (in WSL):       python3 scripts/control.py            # runs the demo in main()
Use as a library:   from control import Drone

SITL exposes TCP 5760/5762/5763; each accepts one client, so this script can
run alongside Mission Planner (use a different port).
Feed release (needs SITL started with FEEDER=1): d.dispense(rate_kg_s, kg), d.release_feed(kg)
"""
import json
import math
import os
import sys
import threading
import time

from pymavlink import mavutil

ML = mavutil.mavlink


class Drone:
    def __init__(self, conn="tcp:127.0.0.1:5760"):
        self.m = mavutil.mavlink_connection(conn)
        self.m.wait_heartbeat(timeout=30)
        self.ts, self.tc = self.m.target_system, self.m.target_component
        self.m.mav.request_data_stream_send(self.ts, self.tc, ML.MAV_DATA_STREAM_ALL, 10, 1)
        self._feed_init()

    # --- feed hopper -----------------------------------------------------
    def _feed_init(self):
        """Feed is split into N detachable chunks in the Gazebo model (feed.json written by gen_bigquad.py)."""
        path = os.path.expanduser("~/simulators/bigquad/feed.json")
        cfg = json.load(open(path)) if os.path.exists(path) else {"chunks": 0, "chunk_kg": 0.0}
        self.feed_chunks, self.chunk_kg, self.released = cfg["chunks"], cfg["chunk_kg"], 0
        self._lock = threading.Lock()
        if self.feed_chunks:
            from gz.msgs10.empty_pb2 import Empty
            from gz.transport13 import Node

            self._empty = Empty
            self._node = Node()
            self._pubs = [self._node.advertise(f"/feed/detach_{i}", Empty) for i in range(self.feed_chunks)]
            time.sleep(1.0)  # let gz-transport discovery finish before the first publish

    def feed_remaining(self):
        """kg of feed still in the hopper."""
        return (self.feed_chunks - self.released) * self.chunk_kg

    def release_feed(self, kg):
        """Drop ~kg of feed now (rounded to whole chunks, bottom of hopper first). Returns kg dropped."""
        if not self.feed_chunks:
            raise RuntimeError("no feeder in the model: start SITL with FEEDER=1")
        with self._lock:
            n = min(round(kg / self.chunk_kg), self.feed_chunks - self.released)
            for _ in range(n):
                self._pubs[self.released].publish(self._empty())
                self.released += 1
        return n * self.chunk_kg

    def dispense(self, rate_kg_s, kg=None, wait=True):
        """Release feed continuously at rate_kg_s until kg is gone (default: everything left).

        wait=False runs it in a background thread (returns the Thread) so you can keep flying."""
        total = self.feed_remaining() if kg is None else min(kg, self.feed_remaining())
        n = round(total / self.chunk_kg)

        def run():
            t0 = time.time()
            for k in range(n):
                time.sleep(max(0.0, t0 + k * self.chunk_kg / rate_kg_s - time.time()))
                self.release_feed(self.chunk_kg)

        th = threading.Thread(target=run, daemon=True)
        th.start()
        if wait:
            th.join()
        return th

    # --- low level -------------------------------------------------------
    def cmd(self, c, *p):
        self.m.mav.command_long_send(self.ts, self.tc, c, 0, *(list(p) + [0] * (7 - len(p))))

    def set_mode(self, name):
        self.m.set_mode(self.m.mode_mapping()[name])

    def armed(self):
        """True/False from the next heartbeat, or None if none arrived (sim can run slower than real time)."""
        hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
        return None if hb is None else bool(hb.base_mode & ML.MAV_MODE_FLAG_SAFETY_ARMED)

    def pos(self):
        """(north, east, up) in metres, relative to home."""
        p = self.m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        return (p.x, p.y, -p.z) if p else None

    # --- flight ----------------------------------------------------------
    def wait_ready(self, timeout=90):
        t0 = time.time()
        while time.time() - t0 < timeout:
            s = self.m.recv_match(type="STATUSTEXT", blocking=True, timeout=2)
            if s and "is using GPS" in s.text:
                time.sleep(3)
                return
        raise TimeoutError("EKF/GPS not ready")

    def arm(self, timeout=120):
        self.set_mode("GUIDED")
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.cmd(ML.MAV_CMD_COMPONENT_ARM_DISARM, 1, 21196)  # force arm
            if self.armed():
                return
            time.sleep(1)
        raise TimeoutError("arming failed")

    def takeoff(self, alt, tol=0.95, timeout=90):
        self.cmd(ML.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self.pos()
            if p and p[2] >= tol * alt:
                return p
        raise TimeoutError("takeoff did not reach altitude")

    def goto(self, north, east, up, yaw=None, tol=1.0, timeout=120):
        """Fly to a local position (NED origin = home) and wait until within tol."""
        mask = 0b0000111111111000 if yaw is None else 0b0000101111111000
        for _ in range(int(timeout * 5)):
            self.m.mav.set_position_target_local_ned_send(
                0, self.ts, self.tc, ML.MAV_FRAME_LOCAL_NED, mask,
                north, east, -up, 0, 0, 0, 0, 0, 0, math.radians(yaw or 0), 0)
            p = self.pos()
            if p and math.dist(p, (north, east, up)) < tol:
                return p
            time.sleep(0.2)
        raise TimeoutError("goto timeout")

    def velocity(self, vn, ve, vd=0.0, duration=1.0):
        """Command a NED velocity (m/s) for `duration` seconds (resent at 5 Hz)."""
        t_end = time.time() + duration
        while time.time() < t_end:
            self.m.mav.set_position_target_local_ned_send(
                0, self.ts, self.tc, ML.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
                0, 0, 0, vn, ve, vd, 0, 0, 0, 0, 0)
            time.sleep(0.2)

    def land(self, timeout=180):
        self.set_mode("LAND")
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.armed() is False:  # ArduPilot auto-disarms after touchdown
                return self.pos()
        raise TimeoutError("landing timeout")


def main():
    conn = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5760"
    d = Drone(conn)
    print("connected; waiting for EKF...")
    d.wait_ready()
    d.arm()
    print("armed, taking off to 20 m")
    print("reached", d.takeoff(20))
    for n, e in [(30, 0), (30, 30), (0, 30), (0, 0)]:
        print(f"goto N={n} E={e} ->", d.goto(n, e, 20, tol=2.0))
    print("landing")
    print("landed", d.land())


if __name__ == "__main__":
    main()
