# Control modes

- **Expressive (available):** scripted joint targets with PD motor control and a temporary support weld. MuJoCo still simulates the joints, contacts and gravity. **Not autonomous balance.** See `bandit_tools/bandit_final_mujoco_wave.py`.
- **Physics (planned as separate example):** unsupport the floating base for gravity, contacts and stability experiments. No independent balancing is claimed.
- **Locomotion (future):** integrate and validate an RP1 standing/walking policy; no policy is bundled in v0.1.
