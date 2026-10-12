import argparse
import json
import math
import time

import numpy as np
from gz.msgs10.wind_pb2 import Wind
from gz.transport13 import Node

Z0 = 2e-4
V_REF = 12.0

p = argparse.ArgumentParser()
p.add_argument("--world", default="bigquad_runway")
p.add_argument("--mean", type=float, default=11.0)
p.add_argument("--dir", type=float, default=25.0)
p.add_argument("--alt", type=float, default=20.0)
p.add_argument("--ti", type=float, default=0.12)
p.add_argument("--tau", type=float, default=10.0)
p.add_argument("--rate", type=float, default=10.0)
p.add_argument("--duration", type=float, default=600.0)
p.add_argument("--ramp", type=float, default=10.0)
p.add_argument("--seed", type=int, default=1)
p.add_argument("--era5", default="")
p.add_argument("--pick", default="")
p.add_argument("--linear", action="store_true")
p.add_argument("--field", default="", help="CFD wind field .npz from wind_cfd/postprocess.py (position-dependent wind)")
p.add_argument("--mav", default="tcp:127.0.0.1:5762", help="MAVLink port used to read the vehicle position for --field")
p.add_argument("--ncdr", default="", help="wind_data/xiyu_ncdr.json (NCDR station observation, see wind_data/ncdr_fetch.py)")
p.add_argument("--window", default="max_48h", help="NCDR window: max_1h|3h|6h|12h|24h|48h")
p.add_argument("--gust-factor", type=float, default=3.0, help="peak factor: gust = mean + gust_factor * sigma")
a = p.parse_args()

if a.era5 and a.pick:
    h = json.load(open(a.era5))["hourly"]
    v = np.array([np.nan if x is None else x for x in h["wind_speed_10m"]])
    d = np.array([np.nan if x is None else x for x in h["wind_direction_10m"]])
    mon = np.array([int(s[5:7]) for s in h["time"]])
    w = np.isin(mon, [10, 11, 12, 1, 2, 3])
    q = {"p50": 50, "p90": 90, "p99": 99}[a.pick]
    target = np.nanpercentile(v[w], q)
    i = np.nanargmin(np.where(w, np.abs(v - target), np.inf))
    a.mean, a.dir = float(v[i]), float(d[i])
    print(f"era5 {a.pick}: {h['time'][i]} v10={a.mean:.2f} dir_from={a.dir:.0f}", flush=True)

a.gust = 0.0
if a.ncdr:
    w = json.load(open(a.ncdr))["windows"][a.window]
    a.mean, a.gust = w["mean"], w["gust"]  # station anemometer height is taken as 10 m
    a.ti = (a.gust - a.mean) / a.gust_factor / a.mean
    print(f"ncdr {a.window}: mean={a.mean} m/s gust={a.gust} m/s -> sigma10={(a.gust - a.mean) / a.gust_factor:.2f} m/s (TI {a.ti:.3f})", flush=True)

field = None
if a.field:
    from pymavlink import mavutil

    f = np.load(a.field)
    field = {k: f[k] for k in f.files}
    dd = abs((float(field["dir_from"]) - a.dir + 180) % 360 - 180)
    if dd > 10:
        print(f"WARNING: CFD field is for wind from {float(field['dir_from']):.0f} deg but requested {a.dir:.0f} deg; using the CFD direction", flush=True)
    a.dir = float(field["dir_from"])
    s = a.mean / float(field["uref"])  # CFD ran with Uref at 10 m; U and sqrt(k) scale linearly with wind speed
    mav = None
    for _ in range(120):  # SITL opens its TCP ports only after it has booted
        try:
            mav = mavutil.mavlink_connection(a.mav)
            mav.wait_heartbeat(timeout=10)
        except OSError:
            mav = None
        if mav is not None and mav.target_system != 0:
            break
        time.sleep(2)
    if mav is None or mav.target_system == 0:
        raise SystemExit(f"cannot reach SITL at {a.mav}")
    mav.mav.request_data_stream_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
    here = np.zeros(3)  # east, north, up (m) relative to the takeoff point

    def interp(arr, e, n, u):
        """Trilinear lookup on the (east, north, up) grid, clamped at the edges."""
        idx = []
        for ax, v in zip((field["x"], field["y"], field["z"]), (e, n, u)):
            t = np.clip((v - ax[0]) / (ax[1] - ax[0]), 0, len(ax) - 1)
            i = min(int(t), len(ax) - 2)
            idx.append((i, t - i))
        out = 0.0
        for cx in (0, 1):
            for cy in (0, 1):
                for cz in (0, 1):
                    wgt = (idx[0][1] if cx else 1 - idx[0][1]) * (idx[1][1] if cy else 1 - idx[1][1]) * (idx[2][1] if cz else 1 - idx[2][1])
                    out = out + wgt * arr[idx[0][0] + cx, idx[1][0] + cy, idx[2][0] + cz]
        return out

k = math.log(max(a.alt, 2 * Z0) / Z0) / math.log(10 / Z0)
mean = a.mean * k
fr = math.radians(a.dir)
ux, uy = -math.sin(fr), -math.cos(fr)
sig = a.ti * mean
tf = 1.0  # CFD turbulence scale: makes the free-stream sigma at 10 m match the observed gust spread
if field is not None and a.gust:
    ustar = 0.41 * float(field["uref"]) / math.log((10.0 + Z0) / Z0)
    tf = ((a.gust - a.mean) / a.gust_factor) / (s * math.sqrt(2.0 / 3.0 * ustar**2 / math.sqrt(0.09)))
    print(f"CFD turbulence scale x{tf:.2f} to match observed gusts", flush=True)
rng = np.random.default_rng(a.seed)
dt = 1.0 / a.rate
phi = math.exp(-dt / a.tau)
noise = math.sqrt(1 - phi * phi)
st = np.zeros(3)
aniso = np.array([1.0, 0.8, 0.5])

node = Node()
pub = node.advertise(f"/world/{a.world}/wind", Wind)
if field is None:
    print(f"mean@{a.alt:.0f}m={mean:.2f} m/s sigma={sig:.2f} from {a.dir:.0f}deg", flush=True)
else:
    print(f"CFD field {a.field}: U10={a.mean:.2f} m/s from {a.dir:.0f}deg, position-dependent mean and sigma", flush=True)
time.sleep(1.0)
t0 = time.time()
n = 0
while time.time() - t0 < a.duration:
    t = time.time() - t0
    st = phi * st + noise * rng.standard_normal(3)
    r = min(1.0, t / a.ramp) if a.ramp > 0 else 1.0
    if field is None:
        mx, my, mz = mean * ux, mean * uy, 0.0
    else:
        while True:  # keep only the newest position
            msg = mav.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if msg is None:
                break
            here[:] = msg.y, msg.x, -msg.z
        mx, my, mz = (s * interp(field["U"][..., i], *here) for i in range(3))
        sig = tf * s * math.sqrt(2.0 / 3.0 * max(float(interp(field["k"], *here)), 0.0))
    mh = math.hypot(mx, my) or 1e-6
    hx, hy = mx / mh, my / mh
    along = (mh + st[0] * aniso[0] * sig) * r
    cross = st[1] * aniso[1] * sig * r
    vx = along * hx - cross * hy
    vy = along * hy + cross * hx
    vz = (mz + st[2] * aniso[2] * sig) * r
    sp = math.hypot(vx, vy)
    f = 1.0 if a.linear else sp / V_REF
    m = Wind()
    m.linear_velocity.x, m.linear_velocity.y, m.linear_velocity.z = vx * f, vy * f, vz
    m.enable_wind = True
    pub.publish(m)
    n += 1
    if n % int(10 * a.rate) == 0:
        where = "" if field is None else f" at E{here[0]:6.1f} N{here[1]:6.1f} up{here[2]:5.1f}m"
        print(f"t={t:5.0f}s wind {sp:5.2f} m/s (mean {mh:5.2f}, sigma {sig:4.2f}){where}", flush=True)
    time.sleep(max(0.0, t0 + n * dt - time.time()))
m = Wind()
m.enable_wind = True
pub.publish(m)
