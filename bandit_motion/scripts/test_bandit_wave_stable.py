"""Experiment 002: RP1 pretrained wave on Bandit standalone MuJoCo model.

- Physics-based joint simulation with the standalone Bandit 24-DoF MJCF.
- A kinematically fixed free base (diagnostic only; NOT autonomous balance).
- ONNX controls both arms and waist, while legs hold a stable default pose.
- Original policy and reference NPZ are unchanged.

Run in Bandit_Motion: python scripts/test_bandit_wave_stable.py
"""
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np
import onnxruntime as ort

HERE = Path(__file__).resolve()
WORK = HERE.parents[1]
PACK = HERE.parents[2]
BANDIT_XML = WORK.parent / 'bandit_description' / 'mjcf' / 'bandit.xml'
POLICY = WORK / 'assets' / 'standing' / 'policy_wave.onnx'
MOTION = WORK / 'assets' / 'standing' / 'wave.npz'

# RP1-derived Bandit 24-DoF joint order: left leg (6), right leg (6), waist (2), two arms (5 each).
DEFAULT = np.array([
    -.1, 0., 0., -.3, -.2, 0.,
     .1, 0., 0.,  .3, -.2, 0.,
     0., 0.,
     .2, .2, 0., -1.2, 0.,
    -.2,-.2, 0.,  1.2, 0.,
], dtype=np.float64)
# Use the 24-value RP1 robot.yaml gain order.
KP = np.array([
    120,120,90,120,40,40,
    120,120,90,120,40,40,
    150,150,
    30,30,20,30,20,
    30,30,20,30,20,
], dtype=np.float64)
KD = np.array([
    9,9,6,9,2,2,
    9,9,6,9,2,2,
    12.5,12.5,
    2.5,2.5,1.5,2.5,1.5,
    2.5,2.5,1.5,2.5,1.5,
], dtype=np.float64)
# Diagnostic-only leg stabilization. These are not RoboParty-trained gains.
LEG_KP_MULTIPLIER = 1.0
LEG_KD_MULTIPLIER = 2.0
POLICY_DT = 0.02  # 50 Hz
PHYSICS_DT = 0.004  # 250 Hz
SUBSTEPS = 5


def main():
    for path in (BANDIT_XML, POLICY, MOTION):
        if not path.is_file():
            raise FileNotFoundError(path)
    for name, x in [('DEFAULT',DEFAULT), ('KP',KP), ('KD',KD)]:
        if x.shape != (24,):
            raise ValueError(f'{name} must have 24 values; found {x.shape}')
    model = mujoco.MjModel.from_xml_path(str(BANDIT_XML))
    if model.nu != 24 or model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
        raise RuntimeError('Expected 24 hinge actuators and a floating base')
    model.opt.timestep = PHYSICS_DT
    data = mujoco.MjData(model)
    qidx = np.empty(24, dtype=int)
    vidx = np.empty(24, dtype=int)
    for i in range(24):
        jid = int(model.actuator_trnid[i, 0])
        if jid < 0 or model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            raise RuntimeError(f'Actuator {i} does not drive a hinge joint')
        qidx[i] = model.jnt_qposadr[jid]
        vidx[i] = model.jnt_dofadr[jid]
    expected_joint_names = [
        'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
        'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
        'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
        'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
        'waist_roll_joint', 'waist_yaw_joint',
        'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
        'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_joint',
        'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
        'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_joint',
    ]
    actual_joint_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT,
                          int(model.actuator_trnid[i, 0]))
        for i in range(model.nu)
    ]
    if actual_joint_names != expected_joint_names:
        for i, (actual, expected) in enumerate(zip(actual_joint_names, expected_joint_names)):
            if actual != expected:
                print(f'Joint mapping mismatch at {i}: found {actual!r}, expected {expected!r}')
        raise RuntimeError('Bandit actuator order differs from RP1 wave policy; aborting to protect the test')
    if not np.allclose(model.actuator_gear[:, 0], 1.):
        raise RuntimeError('Expected unit motor gearing')
    with np.load(MOTION, allow_pickle=False) as f:
        ref_q = f['joint_pos'].astype(np.float32)
        ref_v = f['joint_vel'].astype(np.float32)
        fps = int(f['fps'][0])
    if ref_q.ndim != 2 or ref_q.shape[1] != 24 or ref_q.shape != ref_v.shape or fps != 50:
        raise ValueError('Unexpected reference motion dimensions/fps')
    session = ort.InferenceSession(str(POLICY), providers=['CPUExecutionProvider'])
    inp, out = session.get_inputs()[0], session.get_outputs()[0]
    if inp.shape != [1, 126] or out.shape != [1, 24]:
        raise ValueError(f'Unexpected policy dimensions: {inp.shape} -> {out.shape}')

    mujoco.mj_resetData(model, data)
    data.qpos[qidx] = DEFAULT
    # Save starting floating-base pose, then kinematically fix just that pose.
    # The joints remain fully simulated by MuJoCo.
    base_q = data.qpos[:7].copy()
    if base_q[3] == 0 and np.linalg.norm(base_q[3:7]) == 0:
        base_q[3:7] = [1, 0, 0, 0]
    data.qpos[:7] = base_q
    data.qvel[:6] = 0
    mujoco.mj_forward(model, data)

    kp = KP.copy()
    kd = KD.copy()
    kp[:12] *= LEG_KP_MULTIPLIER
    kd[:12] *= LEG_KD_MULTIPLIER
    last_action = np.zeros(24, dtype=np.float32)
    target = DEFAULT.copy()
    ref_frames = len(ref_q)
    total_policy_frames = ref_frames + 30  # short hold at last wave pose
    max_leg_deviation = 0.0

    print('BANDIT | RoboParty pretrained wave | stable-leg experiment')
    print('Model:', BANDIT_XML)
    print('Root is kinematically fixed: NOT autonomous balance.')
    print('Legs: held at default pose with PD. Waist + arms: pretrained ONNX.')
    print(f'Reference: {ref_frames} frames at {fps} Hz ({ref_frames/fps:.2f}s)')

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.2
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -18
        viewer.cam.lookat[:] = [0, 0, 0.7]
        for step in range(total_policy_frames * SUBSTEPS):
            if not viewer.is_running():
                break
            t0 = time.perf_counter()
            if step % SUBSTEPS == 0:
                frame = min(step // SUBSTEPS, ref_frames - 1)
                omega = data.sensor('angular-velocity').data.copy()
                quat = data.sensor('orientation').data.copy()
                rot = np.empty(9, dtype=np.float64)
                mujoco.mju_quat2Mat(rot, quat)
                gravity_body = rot.reshape(3, 3).T @ np.array([0.,0.,-1.])
                obs = np.concatenate((
                    ref_q[frame], ref_v[frame], omega, gravity_body,
                    data.qpos[qidx] - DEFAULT, data.qvel[vidx], last_action,
                )).astype(np.float32)
                if obs.shape != (126,) or not np.isfinite(obs).all():
                    raise RuntimeError('Invalid policy observation')
                raw = session.run([out.name], {inp.name: np.clip(obs, -100,100)[None,:]})[0][0]
                last_action = np.clip(raw, -100.,100.).astype(np.float32)
                if not np.isfinite(last_action).all():
                    raise RuntimeError('Invalid policy output')
                target = DEFAULT + 0.25 * last_action
                # Intentionally suppress pretrained leg commands for this
                # supported-base *gesture isolation* test only.
                target[:12] = DEFAULT[:12]
                if frame % 25 == 0 and step // SUBSTEPS == frame:
                    print(f'Frame {frame:3d}/{ref_frames-1}  '
                          f't={data.time:.2f}s  '
                          f'leg_err={np.max(np.abs(data.qpos[qidx[:12]]-DEFAULT[:12])):.3f} rad')

            torques = kp * (target - data.qpos[qidx]) - kd * data.qvel[vidx]
            data.ctrl[:] = np.clip(torques,
                                   model.actuator_ctrlrange[:,0],
                                   model.actuator_ctrlrange[:,1])
            mujoco.mj_step(model, data)
            # Pin the floating base ONLY. This makes the diagnostic comparable
            # to the previously successful supported-base run.
            data.qpos[:7] = base_q
            data.qvel[:6] = 0
            mujoco.mj_forward(model, data)
            max_leg_deviation = max(max_leg_deviation,
                float(np.max(np.abs(data.qpos[qidx[:12]] - DEFAULT[:12]))))
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                print('STOP: non-finite simulation state')
                break
            if step % 5 == 0:
                viewer.sync()
            remaining = PHYSICS_DT - (time.perf_counter() - t0)
            if remaining > 0:
                time.sleep(remaining)
        print(f'Final policy frame: {min(step//SUBSTEPS, ref_frames-1)}/{ref_frames-1}')
        print(f'Max leg deviation: {max_leg_deviation:.4f} rad')
        print('Experiment finished.')


if __name__ == '__main__':
    main()
