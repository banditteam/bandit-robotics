import math

"""Bandit RP1 custom upper body: supported expressive wave.

Development mode: a MuJoCo weld anchors the robot base in place, so this
is NOT autonomous balance. Limb joints run under physics with PD motor torques.
The original RP1 XML and the pretrained policy are not modified.

Save in RP1_Bandit_Robot/bandit_tools and run:
  py -3.12 .\bandit_tools\bandit_final_mujoco_wave.py
"""
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco
import mujoco.viewer
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT / 'RP1_description' / 'mjcf' / 'robot_24dof.xml'
TEMP = SOURCE.parent / '_bandit_final_wave_scene.xml'
FPS = 50

# Load all visuals from the *final URDF*, with the correct link attachments.
# Only original RP1 visual geoms are modified; collision and dynamics untouched.
FINAL_URDF = PROJECT / 'bandit_description' / 'bandit_final_v2.urdf'
urtree = ET.parse(FINAL_URDF)
urobot = urtree.getroot()
root = ET.parse(SOURCE).getroot()
asset = root.find('asset')
worldbody = root.find('worldbody')
if asset is None or worldbody is None:
    raise RuntimeError('Original MJCF missing asset or worldbody')

original_model = mujoco.MjModel.from_xml_path(str(SOURCE))
body_xml = {b.get('name'): b for b in worldbody.iter('body') if b.get('name')}
urdf_links = {l.get('name'): l for l in urobot.findall('link')}


def visual_entries(link):
    return [v for v in link.findall('visual') if v.find('geometry/mesh') is not None]


def rgba_for(v):
    color = v.find('material/color')
    if color is not None:
        return color.get('rgba')
    return '0.86 0.84 0.79 1'


def visual_xml_geom(body_name):
    """Find original noncolliding mesh visual, using compiled properties."""
    b = body_xml.get(body_name)
    if b is None:
        raise RuntimeError(f'Body absent from MJCF: {body_name}')
    bid = mujoco.mj_name2id(original_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    direct = b.findall('geom')
    compiled = [i for i in range(original_model.ngeom)
                if int(original_model.geom_bodyid[i]) == bid]
    if len(direct) != len(compiled):
        raise RuntimeError(f'Geom mapping mismatch on {body_name}')
    options = [(g, i) for g, i in zip(direct, compiled)
               if g.get('mesh')
               and int(original_model.geom_contype[i]) == 0
               and int(original_model.geom_conaffinity[i]) == 0]
    if len(options) != 1:
        raise RuntimeError(f'Expected one original visual on {body_name}, found {len(options)}')
    return options[0][0]


# Match URDF visual materials to the matching original RP1 MJCF visuals.
colored = 0
for link_name, link in urdf_links.items():
    if link_name == 'waist_yaw_link' or link_name not in body_xml:
        continue
    visuals = visual_entries(link)
    if len(visuals) != 1:
        continue
    visual_xml_geom(link_name).set('rgba', rgba_for(visuals[0]))
    colored += 1

# Replace only the upper-body *visual*; its collision geoms remain unchanged.
waist = body_xml.get('waist_yaw_link')
if waist is None or 'waist_yaw_link' not in urdf_links:
    raise RuntimeError('waist_yaw_link missing')
original_visual = visual_xml_geom('waist_yaw_link')
waist.remove(original_visual)

mesh_folder = PROJECT / 'bandit_description' / 'meshes'
texture_file = mesh_folder / 'textures' / 'bandit_head_basecolor.png'
if not texture_file.exists():
    raise FileNotFoundError(texture_file)
ET.SubElement(asset, 'texture', {
    'name': 'bandit_atlas_tex', 'type': '2d',
    'file': texture_file.resolve().as_posix()})
ET.SubElement(asset, 'material', {
    'name': 'bandit_atlas_mat', 'texture': 'bandit_atlas_tex',
    'rgba': '1 1 1 1', 'specular': '0.15', 'shininess': '0.15'})


def urdf_rpy_to_quat(rpy):
    """Convert URDF roll/pitch/yaw in radians to MuJoCo wxyz quaternion."""
    roll, pitch, yaw = map(float, rpy.split())

    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return f"{w:.16g} {x:.16g} {y:.16g} {z:.16g}"


added = []
for i, visual in enumerate(visual_entries(urdf_links['waist_yaw_link'])):
    filename = visual.find('geometry/mesh').get('filename', '')
    mesh_file = mesh_folder / Path(filename.replace('\\', '/')).name
    if not mesh_file.exists():
        raise FileNotFoundError(mesh_file)
    if mesh_file.suffix.lower() == '.stl':
        obj_file = mesh_file.with_suffix('.obj')
        if obj_file.is_file():
            print(f'Using OBJ instead of STL: {obj_file.name}')
            mesh_file = obj_file
        else:
            raise RuntimeError(
                f'Please convert STL to OBJ: {mesh_file}'
            )
    mesh_name = f'bandit_final_mesh_{i}'
    ET.SubElement(asset, 'mesh', {
        'name': mesh_name, 'file': mesh_file.resolve().as_posix()})
    origin = visual.find('origin')
    xyz = '0 0 0' if origin is None else origin.get('xyz', '0 0 0')
    rpy = '0 0 0' if origin is None else origin.get('rpy', '0 0 0')
    attrs = {'name': f'bandit_final_visual_{i}', 'type': 'mesh',
             'mesh': mesh_name, 'pos': xyz, 'quat': urdf_rpy_to_quat(rpy),
             'contype': '0', 'conaffinity': '0', 'group': '2', 'density': '0'}
    if mesh_file.suffix.lower() == '.obj' and mesh_file.name in {
        'bandit_head_textured.obj', 'bandit_chest_plate.obj', 'bandit_tail.obj'}:
        attrs['material'] = 'bandit_atlas_mat'
    else:
        attrs['rgba'] = rgba_for(visual)
    ET.SubElement(waist, 'geom', attrs)
    added.append(mesh_file.name)
print('Bandit final visuals:', ', '.join(added))
print('RP1 link visuals recolored:', colored)
print('RP1 joint, collision, actuator and support physics retained')

if worldbody is None:
    raise RuntimeError('RP1 MJCF has no worldbody')

# === BANDIT STUDIO LIGHTING ===
# Render-only modifications. No changes to simulation physics.

visual_settings = root.find("visual")
if visual_settings is None:
    visual_settings = ET.SubElement(root, "visual")

headlight = visual_settings.find("headlight")
if headlight is None:
    headlight = ET.SubElement(visual_settings, "headlight")

headlight.set("ambient", "0.24 0.24 0.24")
headlight.set("diffuse", "0.50 0.50 0.50")
headlight.set("specular", "0.08 0.08 0.08")

for light_name, position, direction, diffuse in [
    (
        "bandit_studio_key",
        "2 -3 5",
        "-0.35 0.45 -1",
        "0.45 0.45 0.45"
    ),
    (
        "bandit_studio_fill",
        "-2 2 4",
        "0.4 -0.3 -1",
        "0.24 0.26 0.30"
    ),
]:
    if not any(
        light.get("name") == light_name
        for light in worldbody.findall("light")
    ):
        ET.SubElement(worldbody, "light", {
            "name": light_name,
            "pos": position,
            "dir": direction,
            "directional": "true",
            "diffuse": diffuse,
            "specular": "0.15 0.15 0.15",
            "castshadow": "false",
        })

print("Bandit studio lighting enabled")
# === END BANDIT STUDIO LIGHTING ===

upper_body = next((b for b in worldbody.iter('body') if b.get('name') == 'waist_yaw_link'), None)
if upper_body is None:
    raise RuntimeError('MJCF has no waist_yaw_link body; cannot attach upper body')

# Weld the floating base to the world *within the physics solver*, rather
# than resetting qpos each step. Joint dynamics, gravity, and contacts remain on.
# Identify the floating-base body from the XML instead of assuming a name.
free_body = None
for b in worldbody.iter('body'):
    if b.find('freejoint') is not None or any(
        j.get('type') == 'free' for j in b.findall('joint')
    ):
        free_body = b.get('name')
        break
if not free_body:
    raise RuntimeError('Could not identify the RP1 floating-base body')
equality = root.find('equality')
if equality is None:
    equality = ET.SubElement(root, 'equality')
ET.SubElement(equality, 'weld', {
    'name': 'bandit_development_support', 'body1': free_body,
    'solref': '0.008 1.0',
})

ET.SubElement(worldbody, 'geom', {
    'name': 'bandit_bright_floor', 'type': 'plane', 'size': '10 10 0.1',
    'pos': '0 0 0', 'rgba': '0.28 0.30 0.34 1',
    'contype': '1', 'conaffinity': '1'
})
try:
    ET.ElementTree(root).write(TEMP, encoding='utf-8', xml_declaration=True)
    model = mujoco.MjModel.from_xml_path(str(TEMP))
finally:
    TEMP.unlink(missing_ok=True)

data = mujoco.MjData(model)
mujoco.mj_resetData(model, data)
base_pose = data.qpos[:7].copy()
base_pose[3:7] = [1, 0, 0, 0]


def joint_id(name):
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if idx < 0:
        raise RuntimeError(f'Expected RP1 joint missing: {name}')
    return idx


def set_pose(q):
    data.qpos[:] = q
    data.qpos[:7] = base_pose
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)


arms = {}
for side in ('left', 'right'):
    names = [f'{side}_shoulder_{axis}_joint' for axis in ('pitch', 'roll', 'yaw')]
    names.append(f'{side}_elbow_joint')
    ids = [joint_id(name) for name in names]
    qadr = np.array([model.jnt_qposadr[j] for j in ids], dtype=int)
    dadr = np.array([model.jnt_dofadr[j] for j in ids], dtype=int)
    limits = np.array([model.jnt_range[j] for j in ids], dtype=float)
    limited = np.array([model.jnt_limited[j] for j in ids], dtype=bool)
    shoulder_body = int(model.jnt_bodyid[ids[0]])
    # Wrist joint is on the hand's forearm link, and gives us a reliable
    # model-specific end-effector anchor even without a named hand site.
    wrist_joint = joint_id(f'{side}_wrist_joint')
    wrist_body = int(model.jnt_bodyid[wrist_joint])
    arms[side] = dict(names=names, ids=ids, qadr=qadr, dadr=dadr,
                      limits=limits, limited=limited, shoulder_body=shoulder_body,
                      wrist_body=wrist_body)

# Work with actual robot geometry rather than guessing the signs of left and
# right shoulder rotations. Each target is the desired wrist position.
initial = data.qpos.copy()
set_pose(initial)
shoulder_positions = {s: data.xpos[a['shoulder_body']].copy() for s,a in arms.items()}
left_y = shoulder_positions['left'][1]
right_y = shoulder_positions['right'][1]
outward = {'left': np.sign(left_y-right_y), 'right': np.sign(right_y-left_y)}
if outward['left'] == 0:
    outward = {'left': 1.0, 'right': -1.0}


def solve_wrist_pose(start, goal_by_side, iterations=160):
    """Damped-least-squares wrist positioning, respecting hinge joint limits."""
    q = start.copy()
    for _ in range(iterations):
        for side, goal in goal_by_side.items():
            a = arms[side]
            set_pose(q)
            current = data.xpos[a['wrist_body']].copy()
            error = goal - current
            if np.linalg.norm(error) < 0.008:
                continue
            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            mujoco.mj_jacBody(model, data, jacp, jacr, a['wrist_body'])
            J = jacp[:, a['dadr']]
            # Damped least-squares prevents singularity near straight arms.
            delta = J.T @ np.linalg.solve(J @ J.T + 0.018 * np.eye(3), error)
            delta = np.clip(delta, -0.07, 0.07)
            candidate = q[a['qadr']] + 0.65 * delta
            candidate[a['limited']] = np.clip(candidate[a['limited']],
                                               a['limits'][a['limited'], 0] + .015,
                                               a['limits'][a['limited'], 1] - .015)
            q[a['qadr']] = candidate
    set_pose(q)
    for side, goal in goal_by_side.items():
        err = np.linalg.norm(data.xpos[arms[side]['wrist_body']] - goal)
        print(f'  {side} wrist target error: {err*100:.1f} cm')
    return q


# Target offsets are in meters from each shoulder, not guessed joint angles.
# Lower = hands beside torso; High = hands above the shoulder near the head.
rest_goals = {}
high_goals = {}
for side, shoulder in shoulder_positions.items():
    rest_goals[side] = shoulder + np.array([0.025, outward[side]*0.10, -0.33])
    high_goals[side] = shoulder + np.array([0.045, outward[side]*0.14, +0.31])

print('Finding hands-down idle pose from actual RP1 geometry...')
rest_pose = solve_wrist_pose(initial, rest_goals)
print('Finding hands-up pose...')
high_pose = solve_wrist_pose(rest_pose, high_goals)

# A genuine side-to-side overhead wave: move raised wrists LATERALLY instead
# of rocking the hands up and down. Both arms stay elevated.
print('Finding wave-left pose...')
wave_left_goals = {s: target + np.array([0, outward[s]*0.10, 0])
                   for s, target in high_goals.items()}
wave_left = solve_wrist_pose(high_pose, wave_left_goals)
print('Finding wave-right pose...')
wave_right_goals = {s: target + np.array([0, -outward[s]*0.065, 0])
                    for s, target in high_goals.items()}
wave_right = solve_wrist_pose(high_pose, wave_right_goals)

# Note: model may not reach exact targets because of joint limits, geometry,
# or singularities. The printed errors make this visible instead of pretending
# every pose is achievable.
sequence = [
    ('Idle: hands DOWN', rest_pose, 1.5),
    ('Both hands UP overhead', high_pose, 2.0),
    ('Hold hands up', high_pose, 0.6),
    ('Wave outward', wave_left, 0.5),
    ('Wave inward', wave_right, 0.5),
    ('Wave outward', wave_left, 0.5),
    ('Wave inward', wave_right, 0.5),
    ('Wave outward', wave_left, 0.5),
    ('Hands up: finish wave', high_pose, 0.6),
    ('Lower both hands', rest_pose, 2.0),
    ('Rest', rest_pose, 1.0),
]


# --------------------------------------------------------------------------
# Physics playback: joint motors track the same poses that already worked
# visually, while MuJoCo integrates gravitational/contact dynamics.
# --------------------------------------------------------------------------
model.opt.timestep = 0.004

# One motor actuator per hinge joint in this 24-DoF model.
if model.nu != 24:
    raise RuntimeError(f'Expected 24 actuators, got {model.nu}')
q_idx, v_idx = [], []
for motor_id in range(model.nu):
    jid = int(model.actuator_trnid[motor_id, 0])
    if jid < 0 or model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
        raise RuntimeError(f'Unexpected actuator-to-joint mapping: {motor_id}')
    q_idx.append(int(model.jnt_qposadr[jid]))
    v_idx.append(int(model.jnt_dofadr[jid]))
q_idx = np.asarray(q_idx, dtype=int)
v_idx = np.asarray(v_idx, dtype=int)
if not np.allclose(model.actuator_gear[:, 0], 1.0):
    raise RuntimeError('Motor gearing is not 1:1; review physics torque mapping')

# Conservative position servo gains for this supported *development* test.
# These aren't a trained balance controller and must not drive real hardware.
KP = np.full(model.nu, 50.0)
KD = np.full(model.nu, 5.0)
KP[:14] = 100.0       # legs and waist held near idle
KD[:14] = 8.0

# Set the initial pose before stepping, then let MuJoCo take over.
mujoco.mj_resetData(model, data)
data.qpos[:] = rest_pose
mujoco.mj_forward(model, data)


def physics_step(target):
    error = target[q_idx] - data.qpos[q_idx]
    torques = KP * error - KD * data.qvel[v_idx]
    data.ctrl[:] = np.clip(torques,
                           model.actuator_ctrlrange[:, 0],
                           model.actuator_ctrlrange[:, 1])
    mujoco.mj_step(model, data)


def run_target(viewer, beginning, target, duration, label):
    print('BANDIT:', label)
    nsteps = max(1, round(duration / model.opt.timestep))
    for k in range(nsteps):
        if not viewer.is_running():
            return False
        start = time.perf_counter()
        phase = (k + 1) / nsteps
        ease = phase * phase * (3 - 2 * phase)
        commanded = beginning + (target - beginning) * ease
        physics_step(commanded)
        if k % max(1, round(1 / (FPS * model.opt.timestep))) == 0:
            viewer.sync()
        delay = model.opt.timestep - (time.perf_counter() - start)
        if delay > 0:
            time.sleep(delay)
    viewer.sync()
    print(f'  max joint error: {np.max(np.abs(data.qpos[q_idx] - target[q_idx])):.3f} rad')
    return True


print('BANDIT RP1 | Supported expressive wave')
print('First: hands-down -> overhead wave -> hands-down')
print('Development support remains active throughout this demo.')
print('Floor contacts and joint dynamics remain enabled. Original RP1 files unchanged.')

support_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY,
                              'bandit_development_support')
if support_id < 0:
    raise RuntimeError('Temporary weld support not found')

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 3.2
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -18
    viewer.cam.lookat[:] = [0, 0, 0.8]

    # Physically settle motors in the relaxed hands-down pose.
    for k in range(375):
        if not viewer.is_running():
            break
        physics_step(rest_pose)
        if k % 5 == 0:
            viewer.sync()
        time.sleep(model.opt.timestep)

    # One expressive sequence. Everything up to here is the existing
    # tested supported-base motor controller, not a walking policy.
    previous = rest_pose.copy()
    for name, target, duration in sequence:
        if not viewer.is_running():
            break
        if not run_target(viewer, previous, target, duration / 1.5, name):
            break
        previous = target.copy()

    # Hold the rest pose under the existing development weld. This is not autonomous balance.
    if viewer.is_running():
        print('BANDIT: Standing expressive demo complete. Close viewer to exit.')
    while viewer.is_running():
        t0 = time.perf_counter()
        physics_step(rest_pose)
        viewer.sync()
        time.sleep(max(0.0, model.opt.timestep - (time.perf_counter() - t0)))
