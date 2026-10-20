# Robot description

`bandit_description/bandit_final_v2.urdf` is the canonical v0.1 visual URDF. The original 24-DoF MuJoCo model is `RP1_description/mjcf/robot_24dof.xml`.

Bandit head, torso, chest plate, and tail are currently attached to `waist_yaw_link` as visual meshes. The decorative tail and head have **no independent joints** in v0.1. All extra mesh visuals used in MuJoCo are non-colliding; this should not be mistaken for full physical head/tail articulation.
