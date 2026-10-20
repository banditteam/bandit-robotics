# Architecture

The existing v0.1 MuJoCo demo loads `RP1_description/mjcf/robot_24dof.xml`, reads the custom visuals from `bandit_description/bandit_final_v2.urdf`, and builds a temporary scene at runtime. It recolors stock non-colliding geoms, replaces the original upper-body visual with head/torso/chest/tail visuals, adds studio lighting and a floor, and uses a development weld.

The source RP1 MJCF, RP1 collision bodies, and 24 actuator mapping remain unchanged in the checked-in source files. The Bandit decorative meshes are visual-only in the MuJoCo scene. The support weld remains enabled throughout the demonstration; this is not autonomous stabilization.

The v0.1 demo remains in its original `bandit_tools/` path to preserve known-working relative asset resolution. Future extraction into `simulations/` and reusable `controllers/` requires regression testing.
