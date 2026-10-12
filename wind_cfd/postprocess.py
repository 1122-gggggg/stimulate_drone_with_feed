"""Convert the sampled OpenFOAM result into the .npz wind field read by generator/windfeed.py --field.

  python3 wind_cfd/postprocess.py wind_cfd/cases/d022 [out.npz]     # default out: wind_data/xiyu_d022.npz

The file holds, on a regular (east, north, up) grid in metres relative to the takeoff point:
  U[x,y,z,3]  CFD velocity in world axes (east, north, up) for Uref at 10 m   k[x,y,z]  turbulent KE
Grid points the solver has no cell for (inside obstacles) get the undisturbed marine log profile.
"""
import glob
import json
import math
import os
import sys

import numpy as np

case = sys.argv[1]
fr = json.load(open(os.path.join(case, "frame.json")))
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "wind_data", f"xiyu_d{os.path.basename(case.rstrip('/'))[1:]}.npz")

times = sorted(glob.glob(os.path.join(case, "postProcessing", "samples", "*")), key=lambda d: float(os.path.basename(d)))
src = os.path.join(times[-1], "cloud.xy")
print("reading", src)
raw = np.loadtxt(src, comments="#")  # x y z Ux Uy Uz k  (flow frame)
print("rows", len(raw))

ux, uy = fr["ux"], fr["uy"]
vx, vy = -uy, ux
key = lambda x, y, z: (round(x * 1000), round(y * 1000), round(z * 1000))
table = {}
spread = 0.0
for r in raw:
    k_ = key(r[0], r[1], r[2])
    if k_ in table:
        spread = max(spread, float(np.abs(table[k_] - r[3:]).max()))
    table[k_] = r[3:]
print("unique points", len(table), "max disagreement between duplicate rows", spread)

gx, gz = np.array(fr["ground_x"]), np.array(fr["ground_z"])
uref, zref, z0 = fr["uref"], fr["zref"], fr["z0"]
ustar = 0.41 * uref / math.log((zref + z0) / z0)
U = np.zeros((len(gx), len(gx), len(gz), 3))
K = np.zeros((len(gx), len(gx), len(gz)))
missing = 0
for i, e in enumerate(gx):
    for j, n in enumerate(gx):
        fx, fy = round(e * ux + n * uy, 3), round(e * vx + n * vy, 3)  # same 3-decimal rounding as the sample dict
        for m, z in enumerate(gz):
            row = table.get(key(fx, fy, z))
            if row is None:
                missing += 1
                row = np.array([ustar / 0.41 * math.log((z + z0) / z0), 0.0, 0.0, ustar**2 / math.sqrt(0.09)])
            U[i, j, m] = row[0] * np.array([ux, uy, 0]) + row[1] * np.array([vx, vy, 0]) + row[2] * np.array([0, 0, 1])
            K[i, j, m] = row[3]
print(f"grid {U.shape[:3]}, filled with undisturbed profile: {missing}")
np.savez_compressed(out, x=gx, y=gx, z=gz, U=U.astype(np.float32), k=K.astype(np.float32),
                    dir_from=fr["dir_from"], uref=uref)
print("wrote", os.path.abspath(out), f"{os.path.getsize(out) / 1e6:.1f} MB")
