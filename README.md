# Bandit Robotics · v0.1 candidate

Bandit is a custom humanoid meme-mascot simulation and hardware built on the open-source RoboParty RP1 24-DoF robot model.

## Status
- **Expressive demo:** scripted PD joint movement with temporary support weld; finishes standing upright. **Not autonomous balance.**
- **Physics:** MuJoCo gravity, contacts and joint dynamics remain active. Free-body tests are planned separately.
- **Locomotion:** not yet integrated or validated. RP1 upstream policies are not bundled.
- **Hardware:** simulation only. Never run this controller on physical hardware.

## Quick start (Python 3.12, GUI desktop)

```bash
python -m pip install -r requirements.txt
python examples/run_bandit.py
```

The first startup may take time to compile textured, high-detail meshes. Close the MuJoCo viewer to quit.

## Folder layout
- `bandit_description/`: final URDF and linked meshes/textures
- `RP1_description/`: upstream MuJoCo 24-DoF source model and required RP1 meshes
- `bandit_tools/`: bandit expressive demo
- `examples/`: user entry point
- `controllers/`: future reusable expressive, physics, and locomotion interfaces
- `blender/`: Blender contribution guidance; `.blend` master staged locally, not committed by default
- `tests/`: dependency and static portability checks

## Upstream / licensing
Bandit is an open-source robotics project released under the MIT License for its original software and contributions. Certain third-party components, including RoboParty deployment resources, RP1 mechanical designs, and external assets, may be governed by their respective licenses.
