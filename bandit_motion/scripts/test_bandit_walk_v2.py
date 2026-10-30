"""Bandit Motion experiment 003: Bandit pretrained RP1 velocity policy in MuJoCo.

Run from Bandit_Motion:
    python scripts/test_bandit_walk_v2.py --vx 0.40 --duration 8

Exploratory sim2sim test, NOT validated autonomous locomotion or hardware control.
Does not edit the existing wave demos, original RP1 runner, or Bandit MJCF.
"""
from __future__ import annotations

import argparse
from collections import deque
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np
import onnxruntime as ort

WORK = Path(__file__).resolve().parents[1]
PACK = WORK.parent
XML = WORK.parent / "bandit_description" / "mjcf" / "bandit.xml"
POLICY = WORK / "assets" / "locomotion" / "policy_walk.onnx"

# RP1 24-DoF policy/deployment configuration; ordered by actuators.
DEFAULT = np.array([
    -.1, 0, 0, -.3, -.2, 0, .1, 0, 0, .3, -.2, 0,
    0, 0, .2, .2, 0, -1.2, 0, -.2, -.2, 0, 1.2, 0,
], dtype=np.float64)
KP = np.array([
    120, 120, 90, 120, 40, 40,
    120, 120, 90, 120, 40, 40,
    150, 150,
    30, 30, 20, 30, 20,
    30, 30, 20, 30, 20,
], dtype=np.float64)
KD = np.array([
    9, 9, 6, 9, 2, 2,
    9, 9, 6, 9, 2, 2,
    12.5, 12.5,
    2.5, 2.5, 1.5, 2.5, 1.5,
    2.5, 2.5, 1.5, 2.5, 1.5,
], dtype=np.float64)
FIELDS = (
    ("ang_vel", 3), ("gravity_b", 3), ("cmd_vel", 3),
    ("dof_pos", 24), ("dof_vel", 24), ("last_action", 24),
)
DT = 0.004
DECIMATION = 5  # ONNX policy at 50 Hz; MuJoCo at 250 Hz.


def args_parse():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vx", type=float, default=0.20, help="forward m/s")
    parser.add_argument("--vy", type=float, default=0.0, help="lateral m/s")
    parser.add_argument("--yaw", type=float, default=0.0, help="yaw rad/s")
    parser.add_argument("--duration", type=float, default=8.0, help="seconds")
    parser.add_argument("--support-seconds", type=float, default=0.0,
                        help="optional kinematic base hold during startup; NOT balance")
    return parser.parse_args()


def main():
    args = args_parse()
    if args.duration <= 0 or args.support_seconds < 0:
        raise ValueError("Duration must be positive; support-seconds nonnegative")
    for p in (XML, POLICY):
        if not p.is_file():
            raise FileNotFoundError(p)
    for name, values in (("DEFAULT", DEFAULT), ("KP", KP), ("KD", KD)):
        if values.shape != (24,):
            raise RuntimeError(f"{name} does not have 24 elements")

    print(f"Loading Bandit model: {XML}")
    model = mujoco.MjModel.from_xml_path(str(XML))
    floors = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
              for i in range(model.ngeom)
              if model.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE]
    if "bandit_bright_floor" not in floors:
        raise RuntimeError(f"Expected bandit_bright_floor in model, found: {floors}")
    print("Verified Bandit ground: bandit_bright_floor")
    if model.nu != 24 or model.nq < 31 or model.nv < 30:
        raise RuntimeError(f"Unexpected model dimensions: nq={model.nq}, nv={model.nv}, nu={model.nu}")
    model.opt.timestep = DT

    qidx, vidx = [], []
    expected_joint_order = (
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_roll_joint", "waist_yaw_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_joint",
    )
    for i, expected in enumerate(expected_joint_order):
        j = int(model.actuator_trnid[i, 0])
        actual = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if actual != expected or model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            raise RuntimeError(f"Actuator {i}: expected {expected}, found {actual}")
        qidx.append(int(model.jnt_qposadr[j]))
        vidx.append(int(model.jnt_dofadr[j]))
    if not np.allclose(model.actuator_gear[:, 0], 1.0):
        raise RuntimeError("Actuator gearing differs from expected 1:1")
    qidx, vidx = np.asarray(qidx), np.asarray(vidx)

    session = ort.InferenceSession(str(POLICY), providers=["CPUExecutionProvider"])
    inp, out = session.get_inputs()[0], session.get_outputs()[0]
    if inp.shape != [1, 243] or out.shape != [1, 24]:
        raise RuntimeError(f"Expected [1,243]->[1,24], got {inp.shape}->{out.shape}")

    cmd = np.array([np.clip(args.vx, -.4, .6),
                    np.clip(args.vy, -.4, .4),
                    np.clip(args.yaw, -.8, .8)], dtype=np.float32)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    # Bandit includes a development-only world support weld. Release it
    # explicitly so locomotion uses a genuinely free floating base.
    support_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_EQUALITY, "bandit_development_support"
    )
    if support_id < 0:
        raise RuntimeError("Bandit development support weld not found; verify model")
    data.eq_active[support_id] = 0
    print("Bandit equalities:", model.neq)
    for i in range(model.neq):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_EQUALITY, i)
        print(f"  equality {i}: {name!r}, active={bool(data.eq_active[i])}")
    if bool(data.eq_active[support_id]):
        raise RuntimeError("Bandit support weld remains active")
    print("Verified Bandit support weld: DISABLED (free-base)")

    data.qpos[qidx] = DEFAULT
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    initial_root = data.qpos[:7].copy()
    initial_x = float(data.qpos[0])

    # One queue per source, with oldest -> newest entries per source.
    history = {name: deque(maxlen=3) for name, _ in FIELDS}
    last_action = np.zeros(24, dtype=np.float32)
    joint_targets = DEFAULT.copy()
    nsteps = int(args.duration / DT)
    max_torque = 0.0

    print("BANDIT WALK V2 | RP1 pretrained velocity policy")
    print("Physics: 250 Hz | policy: 50 Hz | obs-major history: 243")
    print(f"Command vx={cmd[0]:.2f}, vy={cmd[1]:.2f}, yaw={cmd[2]:.2f}")
    print("WARNING: free-base exploratory test; may fall; no guaranteed balance")
    print(f"Initial root height: {data.qpos[2]:.3f}m")
    if args.support_seconds:
        print(f"Optional temporary base hold: {args.support_seconds:.2f}s")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 3.2
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -18
        viewer.cam.lookat[:] = [0, 0, 0.8]
        for step in range(nsteps):
            if not viewer.is_running():
                print("Viewer closed")
                break
            tic = time.perf_counter()
            if step % DECIMATION == 0:
                omega = data.sensor("angular-velocity").data.copy()
                quat = data.sensor("orientation").data.copy()
                R = np.zeros(9)
                mujoco.mju_quat2Mat(R, quat)
                gravity_b = R.reshape(3, 3).T @ np.array([0.0, 0.0, -1.0])
                segment = {
                    "ang_vel": omega.astype(np.float32),
                    "gravity_b": gravity_b.astype(np.float32),
                    "cmd_vel": cmd,
                    "dof_pos": (data.qpos[qidx] - DEFAULT).astype(np.float32),
                    "dof_vel": data.qvel[vidx].astype(np.float32),
                    "last_action": last_action,
                }
                for name, size in FIELDS:
                    value = segment[name]
                    if value.shape != (size,):
                        raise RuntimeError(f"Bad observation {name}: {value.shape}")
                    if not history[name]:
                        for _ in range(3):
                            history[name].append(value.copy())
                    else:
                        history[name].append(value.copy())
                # RoboParty 'obs_major': all 3 histories of ang_vel, then
                # all 3 of gravity_b, and so on. Oldest frame first.
                obs = np.concatenate([
                    x for name, _ in FIELDS for x in history[name]
                ]).astype(np.float32)
                obs = np.clip(obs, -100.0, 100.0)
                if obs.shape != (243,) or not np.isfinite(obs).all():
                    raise RuntimeError("Invalid policy observation")
                raw = session.run([out.name], {inp.name: obs[None, :]})[0][0]
                last_action = np.clip(raw, -100.0, 100.0).astype(np.float32)
                joint_targets = DEFAULT + 0.25 * last_action
                if step % (DECIMATION * 25) == 0:
                    print(f"t={data.time:5.2f}s  root_z={data.qpos[2]:.3f}m  "
                          f"x={data.qpos[0]:.3f}m  dx={data.qpos[0]-initial_x:+.3f}m  "
                          f"max_action={np.max(np.abs(last_action)):.2f}")

            # Diagnose whether the learned walking targets are actually tracked.
            # Report every 0.5 seconds (125 MuJoCo steps at 250 Hz).
            if step % (DECIMATION * 25) == 0:
                leg_target = joint_targets[:12]
                leg_actual = data.qpos[qidx[:12]]
                tracking_error = leg_target - leg_actual
                print(
                    f"  LEG TRACKING | "
                    f"target change={np.max(np.abs(leg_target - DEFAULT[:12])):.3f} rad | "
                    f"actual change={np.max(np.abs(leg_actual - DEFAULT[:12])):.3f} rad | "
                    f"max error={np.max(np.abs(tracking_error)):.3f} rad"
                )

            torques = KP * (joint_targets - data.qpos[qidx]) - KD * data.qvel[vidx]
            max_torque = max(max_torque, float(np.max(np.abs(torques))))
            data.ctrl[:] = np.clip(torques,
                                   model.actuator_ctrlrange[:, 0],
                                   model.actuator_ctrlrange[:, 1])
            # Diagnostic startup support (kinematic, not a physical constraint).
            # This is only a short initial aid and is removed before judging walking.
            if data.time < args.support_seconds:
                data.qpos[:7] = initial_root
                data.qvel[:6] = 0
                mujoco.mj_forward(model, data)
            mujoco.mj_step(model, data)
            if data.time <= args.support_seconds:
                data.qpos[:7] = initial_root
                data.qvel[:6] = 0
                mujoco.mj_forward(model, data)
            if step % 5 == 0:
                viewer.cam.lookat[:] = data.qpos[:3]
                viewer.sync()
            if (not np.isfinite(data.qpos).all()
                    or not np.isfinite(data.qvel).all()
                    or data.qpos[2] < 0.35):
                print("Stopped: fall / invalid state")
                print(f"t={data.time:.3f}, root={data.qpos[:3]}")
                print(f"max_joint_speed={np.max(np.abs(data.qvel[vidx])):.3f}")
                print(f"max_command_torque={max_torque:.2f}")
                print(f"MuJoCo warnings={data.warning.number}")
                break
            delay = DT - (time.perf_counter() - tic)
            if delay > 0:
                time.sleep(delay)

    print(f"Final displacement x: {data.qpos[0] - initial_x:+.3f}m")
    print("Experiment finished; viewer closed")


if __name__ == "__main__":
    main()
