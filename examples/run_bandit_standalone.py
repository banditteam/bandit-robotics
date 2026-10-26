"""
Bandit standalone expressive wave.

Preserves the original Bandit inverse kinematics,
wave sequence, motor controller, and MuJoCo physics.

Uses Bandit's own MJCF/MJB model.
"""

from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]

MJB = PROJECT / "bandit_description" / "mjcf" / "bandit.mjb"
XML = PROJECT / "bandit_description" / "mjcf" / "bandit.xml"

FPS = 50

if MJB.is_file():
    print("Loading cached Bandit model:", MJB)
    model = mujoco.MjModel.from_binary_path(str(MJB))

elif XML.is_file():
    print("Loading Bandit XML:", XML)
    model = mujoco.MjModel.from_xml_path(str(XML))

else:
    raise FileNotFoundError(
        "Missing bandit.mjb and bandit.xml"
    )

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


print('BANDIT standalone | Original expressive wave')
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
