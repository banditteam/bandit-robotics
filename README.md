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
- `bandit_tools/`: unchanged known-working expressive demo
- `examples/`: user entry point
- `controllers/`: future reusable expressive, physics, and locomotion interfaces
- `blender/`: Blender contribution guidance; `.blend` master staged locally, not committed by default
- `tests/`: dependency and static portability checks

## Limitations
The v0.1 wave demo currently composes the MuJoCo scene dynamically from the upstream model and Bandit URDF. The model is not guaranteed to balance or walk. Detailed CAD/exploded-view assets are developed separately.

## Upstream / licensing
RoboParty's deployment source is GPL-3.0-only according to its LICENSES.md; RP1 mechanical designs and third-party textures may carry other terms. **This candidate is not cleared for publication until every bundled model, material, and asset license has been verified.** See LICENSES.md.
