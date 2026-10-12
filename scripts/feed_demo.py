"""Fly with the feeder, drop the feed in flight, show the thrust drop.

Start SITL first (feeder + Penghu wind):
  GUI=1 FEEDER=1 FEED=5.0 WF="--era5 $ROOT/wind_data/penghu_era5.json --pick p50" bash scripts/serve.sh 0 0 0 20 600
(or: scripts/pen.sh-style env; omit WF for no wind)
Run:  python3 scripts/feed_demo.py
"""
import statistics as st
import sys
import time

from control import Drone


def motor_pwm(d, secs):
    """Mean PWM of motors 1-4 over `secs` seconds."""
    v, t0 = [], time.time()
    while time.time() - t0 < secs:
        s = d.m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=2)
        if s:
            v.append((s.servo1_raw + s.servo2_raw + s.servo3_raw + s.servo4_raw) / 4)
    return st.mean(v)


def main():
    alt = 20.0
    rate = float(sys.argv[1]) if len(sys.argv) > 1 else 0.25  # kg/s
    d = Drone()
    print(f"feed on board: {d.feed_remaining():.2f} kg in {d.feed_chunks} chunks")
    d.wait_ready()
    d.arm()
    d.takeoff(alt)
    d.goto(0, 0, alt, tol=1.0)
    pwm0 = motor_pwm(d, 10)
    print(f"hover, full hopper:  motor PWM {pwm0:.0f}")

    print(f"dispensing at {rate} kg/s while flying north")
    th = d.dispense(rate, wait=False)
    d.goto(60, 0, alt, tol=2.0, timeout=180)
    th.join()
    print(f"hopper empty? remaining {d.feed_remaining():.2f} kg")
    pwm1 = motor_pwm(d, 10)
    print(f"hover, empty hopper: motor PWM {pwm1:.0f}  (change {pwm1 - pwm0:+.0f})")

    d.goto(0, 0, alt, tol=2.0, timeout=180)
    d.land()
    print("landed")


if __name__ == "__main__":
    main()
