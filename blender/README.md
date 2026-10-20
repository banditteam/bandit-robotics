# Blender design contributions

The local builder can stage `Bandit_RP1.blend` under `blender/source/` when present. This folder is Git-ignored because a source Blender file may be large. Publish as a separately licensed release asset or via Git LFS after checking texture and model provenance.

Keep geometry editable, pack images, avoid baked environment/lighting, and do not overwrite the simulation reference model. The detailed exploded-view project is separate and not included in v0.1.
