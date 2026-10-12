"""Build the digital-twin world: same scene, but the drone is a static visual that scripts/mirror.py moves.

Needs a previous run of gen_bigquad.py (the model and bigquad_runway.sdf are derived from its output, so the
look, the feeder, and the obstacles from --site stay identical). Writes, under ~/simulators:
  ardupilot_gazebo/models/bigquad_twin/   static model: no ArduPilotPlugin, no rotor physics, no joints
  ardupilot_gazebo/worlds/bigquad_twin.sdf  no wind, no feed chunk bodies
  bigquad/twin.json                       world / model name and rest height (read by mirror.py)
"""
import json
import os
import re

HOME = os.path.expanduser("~/simulators")
AG = os.path.join(HOME, "ardupilot_gazebo")

model = open(os.path.join(AG, "models", "bigquad", "model.sdf")).read()
model = re.sub(r"<plugin\b[^>]*?/>", "", model)
model = re.sub(r"<plugin\b.*?</plugin>", "", model, flags=re.S)
model = re.sub(r"<joint\b.*?</joint>", "", model, flags=re.S)
model = re.sub(r"<collision\b.*?</collision>", "", model, flags=re.S)
model = re.sub(r"<sensor\b.*?</sensor>", "", model, flags=re.S)
model, n = re.subn(r'<model name="bigquad">', '<model name="bigquad_twin">\n    <static>true</static>', model)
assert n == 1, "model name not found; run gen_bigquad.py first"

mdir = os.path.join(AG, "models", "bigquad_twin")
os.makedirs(mdir, exist_ok=True)
open(os.path.join(mdir, "model.sdf"), "w").write(model)
open(os.path.join(mdir, "model.config"), "w").write(
    '<?xml version="1.0"?>\n<model><name>bigquad_twin</name><version>1.0</version>'
    '<sdf version="1.9">model.sdf</sdf><description>static twin of bigquad</description></model>\n'
)

world = open(os.path.join(AG, "worlds", "bigquad_runway.sdf")).read()
m = re.search(r"<uri>model://bigquad</uri>\s*<pose degrees=\"true\">\S+ \S+ (\S+) ", world)
assert m, "bigquad include not found in bigquad_runway.sdf"
rest_z = float(m.group(1))
world = world.replace("model://bigquad<", "model://bigquad_twin<")
world = world.replace('world name="bigquad_runway"', 'world name="bigquad_twin"')
world = re.sub(r"<plugin\s+filename=\"gz-sim-wind-effects-system\".*?</plugin>", "", world, flags=re.S)
world = re.sub(r"<wind>.*?</wind>", "", world, flags=re.S)
world = re.sub(r"<model name='feed_\d+'>.*?</model>", "", world, flags=re.S)
open(os.path.join(AG, "worlds", "bigquad_twin.sdf"), "w").write(world)

os.makedirs(os.path.join(HOME, "bigquad"), exist_ok=True)
json.dump({"world": "bigquad_twin", "model": "bigquad_twin", "rest_z": rest_z},
          open(os.path.join(HOME, "bigquad", "twin.json"), "w"))
print(f"twin world written: bigquad_twin.sdf, rest_z={rest_z}")
