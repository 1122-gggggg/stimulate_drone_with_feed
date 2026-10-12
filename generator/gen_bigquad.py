import argparse
import json
import math
import os
import shutil

G = 9.81
RHO = 1.2041
CLA, A0 = 4.25, 0.3

p = argparse.ArgumentParser()
p.add_argument("--payload", type=float, default=0.0)
p.add_argument("--wheelbase", type=float, default=1.4)
p.add_argument("--prop-radius", type=float, default=0.381)
p.add_argument("--tmax", type=float, default=150.0)
p.add_argument("--wmax", type=float, default=300.0)
p.add_argument("--wind", type=float, default=0.0)
p.add_argument("--wind-dir", type=float, default=0.0)
p.add_argument("--wind-ext", action="store_true")
p.add_argument("--fuel", type=float, default=3.0)
p.add_argument("--feeder", action="store_true")
p.add_argument("--feed", type=float, default=5.0)
p.add_argument("--feed-chunks", type=int, default=20)
p.add_argument("--site", default="", help="site.json: obstacles are added to the world as static boxes")
p.add_argument("--feeder-cad", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Drone_Feeder_V6_4_Gazebo",
                                                    "Drone_Feeder_V6_4_Gazebo", "models", "drone_feeder_v64"),
               help="drone_feeder_v64 model dir (CAD STL mesh hung under the drone when --feeder)")
p.add_argument("--cda", type=float, default=0.25)
p.add_argument("--out", default=os.path.expanduser("~/simulators/bigquad"))
a = p.parse_args()

AG = os.path.expanduser("~/simulators/ardupilot_gazebo")
d = a.wheelbase / 2 / math.sqrt(2)
rz = 0.09
cp = 0.7 * a.prop_radius
kt = a.tmax / a.wmax**2
area = kt / (RHO * cp**2 * CLA * A0)

comps = [
    (2.5, (0, 0, 0), (0.45, 0.35, 0.15)),
    (3.5, (0, 0, -0.10), (0.25, 0.20, 0.12)),
    (0.9, (0, 0, -0.14), (0.20, 0.14, 0.06)),
    (0.6 + a.fuel, (0, 0, -0.15), (0.30, 0.20, 0.10)),
]
FZ = -0.20  # top of the feeder (drone-side interface) relative to base_link
# V6.4 CAD (metres, its own origin): top interface z=+0.0845, feet bottom z=-0.358, feet at (+-0.237, +-0.170), 0.03 x 0.03 x 0.018
CAD_TOP, CAD_BOTTOM, CAD_FOOT_X, CAD_FOOT_Y, CAD_FOOT_H = 0.0845, 0.4425, 0.237, 0.170, 0.018  # CAD_BOTTOM = top-to-feet height
if a.feeder:
    comps.append((0.561, (0, 0, FZ), (0.30, 0.25, 0.04)))
    comps.append((3.674, (0, 0, FZ - 0.17), (0.30, 0.25, 0.30)))
FEED_KG = a.feed if a.feeder else 0.0
for sx in (-1, 1):
    for sy in (-1, 1):
        comps.append((0.375, (0.25 * sx, 0.25 * sy, 0.03), None))
        comps.append((0.9, (d * sx, d * sy, 0.03), None))
        comps.append((0.3, (0.3 * sx, 0.25 * sy, -0.30), None))
if a.payload > 0:
    comps.append((a.payload, (0, 0, -0.28), (0.30, 0.20, 0.15)))

M = sum(c[0] for c in comps)
com = [sum(c[0] * c[1][i] for c in comps) / M for i in range(3)]
I = [[0.0] * 3 for _ in range(3)]
for m, pos, dim in comps:
    r = [pos[i] - com[i] for i in range(3)]
    r2 = sum(x * x for x in r)
    for i in range(3):
        for j in range(3):
            I[i][j] += m * ((r2 if i == j else 0) - r[i] * r[j])
    if dim:
        x, y, z = dim
        I[0][0] += m / 12 * (y * y + z * z)
        I[1][1] += m / 12 * (x * x + z * z)
        I[2][2] += m / 12 * (x * x + y * y)

rotor_m = 0.3
total_mass = M + FEED_KG + 4 * rotor_m
rotors = [(d, -d, 1), (-d, d, 1), (d, d, -1), (-d, -d, -1)]
L = 2 * a.prop_radius


def rotor_xml(i, x, y, s):
    return f"""    <link name='rotor_{i}'>
      <pose>{x:.4f} {y:.4f} {rz} 0 0 0</pose>
      <inertial>
        <mass>{rotor_m}</mass>
        <inertia><ixx>0.004</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.0145</iyy><iyz>0</iyz><izz>0.018</izz></inertia>
      </inertial>
      <visual name='prop'>
        <geometry><box><size>{L:.3f} 0.045 0.008</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>{'0.8 0.2 0.2' if s > 0 else '0.2 0.2 0.8'} 1</diffuse></material>
      </visual>
    </link>
    <joint name='rotor_{i}_joint' type='revolute'>
      <parent>base_link</parent>
      <child>rotor_{i}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
        <dynamics><damping>0.004</damping></dynamics>
      </axis>
    </joint>
"""


def lift_xml(i, s):
    out = ""
    for sign in (1, -1):
        out += f"""    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <a0>{A0}</a0><alpha_stall>1.4</alpha_stall><cla>{CLA}</cla><cda>0.10</cda><cma>0.0</cma>
      <cla_stall>-0.025</cla_stall><cda_stall>0.0</cda_stall><cma_stall>0.0</cma_stall>
      <area>{area:.5f}</area><air_density>{RHO}</air_density>
      <cp>{sign * cp:.4f} 0 0</cp><forward>0 {sign * s} 0</forward><upward>0 0 1</upward>
      <link_name>rotor_{i}</link_name>
    </plugin>
"""
    return out


def ctrl_xml(i, s):
    return f"""      <control channel="{i}">
        <jointName>rotor_{i}_joint</jointName>
        <useForce>1</useForce>
        <multiplier>{s * a.wmax:.1f}</multiplier>
        <offset>0</offset>
        <servo_min>1100</servo_min>
        <servo_max>1900</servo_max>
        <type>VELOCITY</type>
        <p_gain>0.4</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>
        <i_max>0</i_max><i_min>0</i_min>
        <cmd_max>8</cmd_max><cmd_min>-8</cmd_min>
        <controlVelocitySlowdownSim>1</controlVelocitySlowdownSim>
      </control>
"""


skid_z = -0.45
sdf = f"""<?xml version='1.0'?>
<sdf version="1.9">
  <model name="bigquad">
    <link name='base_link'>
      <enable_wind>true</enable_wind>
      <inertial>
        <pose>{com[0]:.4f} {com[1]:.4f} {com[2]:.4f} 0 0 0</pose>
        <mass>{M:.3f}</mass>
        <inertia><ixx>{I[0][0]:.4f}</ixx><ixy>{I[0][1]:.5f}</ixy><ixz>{I[0][2]:.5f}</ixz><iyy>{I[1][1]:.4f}</iyy><iyz>{I[1][2]:.5f}</iyz><izz>{I[2][2]:.4f}</izz></inertia>
      </inertial>
      <collision name='body'><geometry><box><size>0.45 0.35 0.15</size></box></geometry></collision>
      <collision name='skid_l'><pose>0 0.25 {skid_z} 0 0 0</pose><geometry><box><size>0.9 0.03 0.03</size></box></geometry></collision>
      <collision name='skid_r'><pose>0 -0.25 {skid_z} 0 0 0</pose><geometry><box><size>0.9 0.03 0.03</size></box></geometry></collision>
      <visual name='body'><geometry><box><size>0.45 0.35 0.15</size></box></geometry>
        <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.25 0.25 0.25 1</diffuse></material></visual>
      <visual name='engine_tank'><pose>0 0 -0.12 0 0 0</pose><geometry><box><size>0.35 0.20 0.15</size></box></geometry>
        <material><ambient>0.6 0.4 0 1</ambient><diffuse>0.9 0.6 0 1</diffuse></material></visual>
      <visual name='arm_a'><pose>0 0 0.03 0 0 0.785398</pose><geometry><box><size>{a.wheelbase:.3f} 0.04 0.04</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
      <visual name='arm_b'><pose>0 0 0.03 0 0 -0.785398</pose><geometry><box><size>{a.wheelbase:.3f} 0.04 0.04</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
      <visual name='skid_l'><pose>0 0.25 {skid_z} 0 0 0</pose><geometry><box><size>0.9 0.03 0.03</size></box></geometry></visual>
      <visual name='skid_r'><pose>0 -0.25 {skid_z} 0 0 0</pose><geometry><box><size>0.9 0.03 0.03</size></box></geometry></visual>
      <visual name='strut_fl'><pose>0.3 0.25 -0.225 0 0 0</pose><geometry><cylinder><radius>0.012</radius><length>0.45</length></cylinder></geometry></visual>
      <visual name='strut_fr'><pose>0.3 -0.25 -0.225 0 0 0</pose><geometry><cylinder><radius>0.012</radius><length>0.45</length></cylinder></geometry></visual>
      <visual name='strut_rl'><pose>-0.3 0.25 -0.225 0 0 0</pose><geometry><cylinder><radius>0.012</radius><length>0.45</length></cylinder></geometry></visual>
      <visual name='strut_rr'><pose>-0.3 -0.25 -0.225 0 0 0</pose><geometry><cylinder><radius>0.012</radius><length>0.45</length></cylinder></geometry></visual>
"""
if a.feeder:
    sdf += f"""      <visual name='feeder_cad'><pose>0 0 {FZ - CAD_TOP:.4f} 0 0 0</pose><geometry><mesh><uri>model://drone_feeder_v64/meshes/feeder.stl</uri></mesh></geometry>
        <material><ambient>0.65 0.68 0.72 1</ambient><diffuse>0.65 0.68 0.72 1</diffuse></material></visual>
"""
    for lx in (-CAD_FOOT_X, CAD_FOOT_X):
        for ly in (-CAD_FOOT_Y, CAD_FOOT_Y):
            sdf += f"""      <collision name='feeder_foot_{lx}_{ly}'><pose>{lx} {ly} {FZ - CAD_BOTTOM + CAD_FOOT_H / 2:.4f} 0 0 0</pose><geometry><box><size>0.03 0.03 {CAD_FOOT_H}</size></box></geometry></collision>
"""
if a.payload > 0:
    sdf += """      <visual name='payload'><pose>0 0 -0.28 0 0 0</pose><geometry><box><size>0.30 0.20 0.15</size></box></geometry>
        <material><ambient>0 0.3 0.6 1</ambient><diffuse>0 0.4 0.8 1</diffuse></material></visual>
"""
sdf += """    </link>
    <link name='imu_link'>
      <inertial><mass>0.15</mass><inertia><ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.00002</iyy><iyz>0</iyz><izz>0.00002</izz></inertia></inertial>
      <sensor name="imu_sensor" type="imu">
        <gz_frame_id>imu_link</gz_frame_id>
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on>
        <update_rate>1000.0</update_rate>
      </sensor>
    </link>
    <joint name='imu_joint' type='revolute'>
      <child>imu_link</child><parent>base_link</parent>
      <axis><xyz>0 0 1</xyz><limit><lower>0</lower><upper>0</upper><effort>0</effort><velocity>0</velocity></limit><dynamics><damping>1.0</damping></dynamics></axis>
    </joint>
"""
for i, (x, y, s) in enumerate(rotors):
    sdf += rotor_xml(i, x, y, s)
sdf += '    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>\n'
for i, (_, _, s) in enumerate(rotors):
    sdf += lift_xml(i, s)
for i in range(4):
    sdf += f"""    <plugin filename="gz-sim-apply-joint-force-system" name="gz::sim::systems::ApplyJointForce">
      <joint_name>rotor_{i}_joint</joint_name>
    </plugin>
"""
sdf += """    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <no_time_sync>1</no_time_sync>
      <have_32_channels>0</have_32_channels>
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>imu_link::imu_sensor</imuName>
"""
for i, (_, _, s) in enumerate(rotors):
    sdf += ctrl_xml(i, s)
sdf += "    </plugin>\n"
NCH = max(1, a.feed_chunks)
FEED_H = 0.22 / NCH  # hopper contents are 0.22 m tall, centred at FZ-0.12
FEED_Z = [FZ - 0.12 - 0.11 + (i + 0.5) * FEED_H for i in range(NCH)]  # chunk 0 = bottom, released first
CH_KG = FEED_KG / NCH
if FEED_KG > 0:
    for i in range(NCH):
        sdf += f"""    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>base_link</parent_link>
      <child_model>feed_{i}</child_model>
      <child_link>link</child_link>
      <detach_topic>/feed/detach_{i}</detach_topic>
      <output_topic>/feed/state_{i}</output_topic>
      <suppress_child_warning>true</suppress_child_warning>
    </plugin>
"""
sdf += "  </model>\n</sdf>\n"

mdir = os.path.join(AG, "models", "bigquad")
os.makedirs(mdir, exist_ok=True)
open(os.path.join(mdir, "model.sdf"), "w").write(sdf)
open(os.path.join(mdir, "model.config"), "w").write(
    '<?xml version="1.0"?>\n<model><name>bigquad</name><version>1.0</version>'
    '<sdf version="1.9">model.sdf</sdf><description>28kg class quad</description></model>\n'
)
if a.feeder:
    src = os.path.abspath(a.feeder_cad)
    dst = os.path.join(AG, "models", "drone_feeder_v64")
    stl = os.path.join("meshes", "feeder.stl")
    if not os.path.isfile(os.path.join(src, stl)):
        raise SystemExit(f"feeder CAD model not found: {src} (use --feeder-cad)")
    if not os.path.isfile(os.path.join(dst, stl)) or os.path.getsize(os.path.join(dst, stl)) != os.path.getsize(os.path.join(src, stl)):
        shutil.copytree(src, dst, dirs_exist_ok=True)

cda_total = a.cda + (0.204 if a.feeder else 0.0)
wind_k = 0.5 * RHO * cda_total * 12.0 / M
wind_factor = math.sqrt(wind_k)
wind_xml = ""
if a.wind_ext:
    wind_xml = f"""
    <wind><linear_velocity>0 0 0</linear_velocity></wind>
    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>{wind_factor:.5f}</force_approximation_scaling_factor>
    </plugin>
"""
elif a.wind > 0:
    wd = math.radians(a.wind_dir)
    wind_xml = f"""
    <wind><linear_velocity>{a.wind * math.cos(wd):.3f} {a.wind * math.sin(wd):.3f} 0</linear_velocity></wind>
    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>{wind_factor:.5f}</force_approximation_scaling_factor>
      <horizontal>
        <magnitude>
          <time_for_rise>10</time_for_rise>
          <sin><amplitude_percent>0.15</amplitude_percent><period>13</period></sin>
          <noise type="gaussian"><mean>0</mean><stddev>{0.06 * a.wind:.3f}</stddev></noise>
        </magnitude>
        <direction>
          <time_for_rise>30</time_for_rise>
          <sin><amplitude>8</amplitude><period>25</period></sin>
          <noise type="gaussian"><mean>0</mean><stddev>0.03</stddev></noise>
        </direction>
      </horizontal>
      <vertical>
        <noise type="gaussian"><mean>0</mean><stddev>{0.04 * a.wind:.3f}</stddev></noise>
      </vertical>
    </plugin>
"""
    wind_xml = wind_xml.replace(
        "<magnitude>", f"<magnitude>\n          <time_for_rise>10</time_for_rise>", 0
    )

world = open(os.path.join(AG, "worlds", "iris_runway.sdf")).read()
world = world.replace("iris_with_gimbal", "bigquad")
spawn_z = 0.5 if not a.feeder else CAD_BOTTOM - FZ + 0.03  # feet 3 cm above ground
world = world.replace('<pose degrees="true">0 0 0.195 0 0 90</pose>', f'<pose degrees="true">0 0 {spawn_z:.3f} 0 0 90</pose>')
world = world.replace('world name="iris_runway"', 'world name="bigquad_runway"')
feed_xml = ""
if FEED_KG > 0:
    sx, sy, sz = 0.28, 0.23, 0.9 * FEED_H  # slightly thinner than the pitch so stacked chunks never touch
    ix, iy, iz = (CH_KG / 12 * v for v in (sy * sy + sz * sz, sx * sx + sz * sz, sx * sx + sy * sy))
    for i in range(NCH):
        feed_xml += f"""
    <model name='feed_{i}'>
      <pose>0 0 {spawn_z + FEED_Z[i]:.4f} 0 0 0</pose>
      <link name='link'>
        <inertial><mass>{CH_KG:.4f}</mass><inertia><ixx>{ix:.6f}</ixx><iyy>{iy:.6f}</iyy><izz>{iz:.6f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name='c'><geometry><box><size>{sx} {sy} {sz:.4f}</size></box></geometry></collision>
        <visual name='v'><geometry><box><size>{sx} {sy} {sz:.4f}</size></box></geometry>
          <material><ambient>0.55 0.35 0.1 1</ambient><diffuse>0.75 0.5 0.15 1</diffuse></material></visual>
      </link>
    </model>
"""
obs_xml = ""
if a.site:
    for o in json.load(open(a.site))["obstacles"]:
        ox, oy = o["center"]
        lx, ly, lz = o["size"]
        obs_xml += f"""
    <model name='obs_{o["name"]}'>
      <static>true</static>
      <pose degrees="true">{ox} {oy} {lz / 2} 0 0 {o.get("yaw", 0)}</pose>
      <link name='link'>
        <collision name='c'><geometry><box><size>{lx} {ly} {lz}</size></box></geometry></collision>
        <visual name='v'><geometry><box><size>{lx} {ly} {lz}</size></box></geometry>
          <material><ambient>0.8 0.2 0.1 1</ambient><diffuse>0.9 0.3 0.1 1</diffuse></material></visual>
      </link>
    </model>
"""
world = world.replace("</world>", wind_xml + feed_xml + obs_xml + "\n  </world>")
open(os.path.join(AG, "worlds", "bigquad_runway.sdf"), "w").write(world)

hover_frac = math.sqrt(total_mass * G / (4 * a.tmax))
rot_scale = M / 18.0
parm = f"""FRAME_CLASS	1
FRAME_TYPE	1
MOT_THST_EXPO	1.0
MOT_THST_HOVER	{hover_frac:.3f}
MOT_SPIN_ARM	0.05
MOT_SPIN_MIN	0.08
MOT_BAT_VOLT_MAX	33.6
MOT_BAT_VOLT_MIN	26.4
BATT_MONITOR	4
BATT_CAPACITY	4000
INS_GYRO_FILTER	10
ATC_ACCEL_P_MAX	18000
ATC_ACCEL_R_MAX	18000
ATC_ACCEL_Y_MAX	4000
ATC_RAT_RLL_P	0.12
ATC_RAT_RLL_I	0.12
ATC_RAT_RLL_D	0.004
ATC_RAT_RLL_FLTD	10
ATC_RAT_RLL_FLTT	10
ATC_RAT_PIT_P	0.12
ATC_RAT_PIT_I	0.12
ATC_RAT_PIT_D	0.004
ATC_RAT_PIT_FLTD	10
ATC_RAT_PIT_FLTT	10
ATC_RAT_YAW_P	0.6
ATC_RAT_YAW_I	0.06
ATC_RAT_YAW_FLTE	2
ATC_ANG_RLL_P	4.5
ATC_ANG_PIT_P	4.5
ANGLE_MAX	3000
WPNAV_SPEED	1500
WPNAV_ACCEL	250
WPNAV_SPEED_UP	300
WPNAV_SPEED_DN	200
PILOT_SPEED_UP	300
PSC_ACCZ_P	0.3
PSC_ACCZ_I	0.6
FENCE_ALT_MAX	1000
SIM_SPEEDUP	1
"""
open(os.path.join(a.out, "bigquad.parm"), "w").write(parm)

json.dump({"chunks": NCH if FEED_KG > 0 else 0, "chunk_kg": CH_KG, "total_kg": FEED_KG}, open(os.path.join(a.out, "feed.json"), "w"))

print(f"total_mass={total_mass:.2f}kg (base {M:.2f}+rotors {4*rotor_m:.1f}) payload={a.payload}kg fuel={a.fuel}kg")
print(f"COM=({com[0]:.3f},{com[1]:.3f},{com[2]:.3f}) Ixx={I[0][0]:.3f} Iyy={I[1][1]:.3f} Izz={I[2][2]:.3f}")
print(f"kT={kt:.3e} blade_area={area:.4f} cp={cp:.3f}  Tmax/rotor={a.tmax}N  T/W={4*a.tmax/(total_mass*G):.2f}")
print(f"hover_throttle~{hover_frac:.3f} wind_k={wind_k:.4f}")
print(f"feed={FEED_KG}kg in {NCH if FEED_KG > 0 else 0} detachable chunks of {CH_KG:.3f}kg")
