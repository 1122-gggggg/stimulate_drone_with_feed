"""Build + mesh + solve one wind direction in the OpenFOAM-11 docker image (stdlib only).

  python wind_cfd/run_cfd.py --dir 22                # -> wind_cfd/cases/d022
  python wind_cfd/run_cfd.py --dir 22 --stage mesh   # stop after meshing
Then convert the sampled field for the simulator (needs numpy; run in WSL):
  python3 wind_cfd/postprocess.py wind_cfd/cases/d022
"""
import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
IMAGE = "openfoam/openfoam11-paraview510"

p = argparse.ArgumentParser()
p.add_argument("--dir", type=float, required=True, help="direction the wind comes FROM (deg)")
p.add_argument("--site", default=os.path.join(HERE, "site.json"))
p.add_argument("--np", type=int, default=8)
p.add_argument("--cell", type=float, default=10.0)
p.add_argument("--level", type=int, default=3)
p.add_argument("--stage", choices=["mesh", "all"], default="all")
a = p.parse_args()

case = os.path.join(HERE, "cases", f"d{int(round(a.dir)) % 360:03d}")
shutil.rmtree(case, ignore_errors=True)  # never mix results of an older run into this one
subprocess.run(
    [sys.executable, os.path.join(HERE, "make_case.py"), "--site", a.site, "--dir", str(a.dir), "--out", case,
     "--np", str(a.np), "--cell", str(a.cell), "--level", str(a.level)],
    check=True,
)

has_obst = os.path.exists(os.path.join(case, "constant", "triSurface", "obstacles.stl"))
steps = ["blockMesh > log.blockMesh 2>&1"]
if has_obst:
    steps.append("snappyHexMesh -overwrite > log.snappyHexMesh 2>&1")
steps.append("checkMesh > log.checkMesh 2>&1 || true\ntail -n 12 log.checkMesh")
if a.stage == "all":
    steps += [
        "decomposePar -force > log.decomposePar 2>&1",
        f"mpirun -np {a.np} foamRun -parallel > log.foamRun 2>&1; tail -n 25 log.foamRun",
        "reconstructPar -latestTime -fields '(U k)' > log.reconstructPar 2>&1 || true",
    ]
script = "source /opt/openfoam11/etc/bashrc\ncd /case\nset -e\n" + "\n".join(steps)
cmd = ["docker", "run", "--rm", "--entrypoint", "bash", "-v", f"{case}:/case", IMAGE, "-c", script]
print("running:", " ".join(cmd[:8]), "...", flush=True)
sys.exit(subprocess.run(cmd).returncode)
