# Bandit Robotics

Bandit Robotics is the open-source development platform for Bandit, a community-driven humanoid robot built to evolve through experimentation, collaboration, and shared innovation.

Start with a working robot, bring it into simulation, experiment with movements, develop new controllers, and share what you build.

Whether you're a robotics engineer, developer, researcher, student, or enthusiast, Bandit gives you a foundation to explore robotics and contribute to a growing open-source ecosystem.

**Bandit is a starting point, not a finished robot.**

## What Can You Build?

- **Expressive behaviors** — Make Bandit wave, dance, gesture, react, or develop its own personality.
- **Motion policies** — Experiment with pretrained policies, develop new ONNX-based motion capabilities, or improve existing behaviors.
- **Custom controllers** — Build joint controllers, experiment with feedback control, and develop new robotic capabilities.
- **Physics simulations** — Explore joint dynamics, gravity, collisions, physical interactions, and robotic motion.
- **Locomotion** — Improve walking policies, balancing, velocity tracking, turning, and more advanced movement.
- **Hardware extensions** — Add joints, sensors, actuators, or new components and test their behavior in simulation.
- **Community experiments** — Build new applications, share examples, and extend what Bandit can do.

Your contribution doesn't need to be a major breakthrough. A new gesture, better controller, simulation improvement, or useful experiment can help move the project forward.

## What's Included?

Bandit Robotics provides a standalone simulation and development environment, including robot descriptions, physics simulation, motion policies, and controller implementations.

| Component | Description |
|---|---|
| **Bandit Description** | Custom Bandit robot model, including URDF resources, standalone MJCF, meshes, textures, and joint definitions. |
| **MuJoCo Simulation** | Physics-based simulation using Bandit's standalone MJCF, physical joints, actuators, gravity, and collisions. |
| **Motion Policies** | Pretrained RP1-based ONNX policies for expressive waving and velocity-conditioned locomotion. |
| **Wave Policy** | Coordinated arm and waist movement using an ONNX policy, reference motion data, and joint-space PD control. |
| **Walking Policy** | Locomotion driven by a velocity-conditioned ONNX policy with observation history. |
| **Controller Examples** | Python implementations for inverse kinematics, joint targets, actuator commands, feedback control, and policy inference. |
| **Simulation Tools** | Scripts and utilities for preparing, running, and extending Bandit's simulation environment. |
| **Tests & Documentation** | Resources for validating the robot model, running physical motions, and developing new capabilities. |

Bandit currently uses a **24-degree-of-freedom architecture**.


## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/banditteam/bandit-robotics.git
cd bandit-robotics
```

The repository includes Bandit's standalone MJCF, robot assets, motion policies, and supporting files. No separately downloaded robot model or pretrained policy is required for the included simulations.

### 2. Install Dependencies

Python 3.12 is recommended.

Create a virtual environment:

```bash
python -m venv .venv
```

Activate it.

**Windows (PowerShell):**

```powershell
.\.venv\Scripts\Activate.ps1
```

**Linux/macOS:**

```bash
source .venv/bin/activate
```

Install the base simulation dependencies:

```bash
pip install -r requirements.txt
```

Install the motion-policy dependencies:

```bash
pip install -r bandit_motion/requirements.txt
```

The physical-motion package uses MuJoCo, NumPy, and ONNX Runtime.

### 3. Launch Bandit

Run the standalone MuJoCo reference controller:

```bash
python examples/run_bandit_standalone.py
```

Bandit loads its editable `bandit.xml` model and performs an expressive movement sequence: hands-down standing, raising both arms, overhead waving, and returning to rest.

The controller uses inverse kinematics to calculate target poses and joint actuators to execute them through MuJoCo physics.

A temporary development support constraint stabilizes the robot during the sequence. This is not an autonomous balancing controller.

### 4. Run the Wave Policy

Bandit includes a pretrained ONNX motion policy for coordinated waving.

```bash
python bandit_motion/scripts/test_bandit_wave_stable.py
```

The wave controller combines:

- A pretrained 24-DOF ONNX policy.
- A reference waving trajectory stored in `wave.npz`.
- A 126-dimensional policy observation.
- Joint-space PD control.
- MuJoCo physics at 250 Hz, with policy inference at 50 Hz.

The policy controls expressive arm and waist movements, while the legs are maintained near a default standing pose.

**Current implementation:** The floating base is kinematically supported to isolate the expressive motion. This allows the joint dynamics and pretrained policy to be tested without requiring an autonomous balancing controller.

The policy and reference motion can be found in:

```text
bandit_motion/assets/standing/
├── policy_wave.onnx
└── wave.npz
```

Contributors can experiment with gesture variations, motion references, policy architectures, joint control, and expressive behavior.

### 5. Run the Walking Policy

Bandit also includes a pretrained velocity-conditioned locomotion policy.

Run the validated forward-velocity configuration:

```bash
python bandit_motion/scripts/test_bandit_walk_v2.py --vx 0.40 --duration 8
```

The walking controller combines:

- A pretrained 24-DOF locomotion policy in ONNX format.
- Velocity commands for forward movement, lateral movement, and yaw.
- A 243-dimensional observation assembled from three frames of observation history.
- Joint-space PD control with torque limits.
- MuJoCo physics at 250 Hz, with policy inference at 50 Hz.

The policy is located at:

```text
bandit_motion/assets/locomotion/
└── policy_walk.onnx
```

Additional velocity command options include:

| Parameter | Description |
|---|---|
| `--vx` | Forward velocity command in m/s |
| `--vy` | Lateral velocity command in m/s |
| `--yaw` | Angular velocity command in rad/s |
| `--duration` | Simulation duration in seconds |
| `--support-seconds` | Optional temporary startup support for diagnostic experiments |

**Recommended starting configuration:** `--vx 0.40 --duration 8`, which has been tested successfully with the current Bandit model.

**Development status:** This is an experimental sim-to-sim locomotion implementation. The policy runs with a free-floating base, but reliable autonomous balance, turning, and walking across diverse conditions remain areas for research and contribution. It is not validated for physical hardware deployment.

### 6. Explore the Robot

Bandit's standalone MuJoCo files are located in:

```text
bandit_description/
├── mjcf/
│   └── bandit.xml
└── assets/
    ├── meshes/
    └── textures/
```

Physical motion policies and controllers are organized separately:

```text
bandit_motion/
├── assets/
│   ├── standing/
│   │   ├── policy_wave.onnx
│   │   └── wave.npz
│   └── locomotion/
│       └── policy_walk.onnx
├── configs/
│   ├── rp1_24dof_robot.yaml
│   └── walking.yaml
├── scripts/
│   ├── test_bandit_wave_stable.py
│   └── test_bandit_walk_v2.py
└── requirements.txt
```

Inspect the robot description, explore the actuators and joint hierarchy, study the policy implementations, or create your own controllers and motion experiments.

The editable MJCF is the source of truth for this MuJoCo simulation. For the separately maintained robot description and URDF resources, visit the [Bandit Description](https://github.com/banditteam) repository through the Bandit GitHub organization.

## Motion Policies & Locomotion

Bandit is designed to be an open environment for developing and sharing robotic intelligence, not simply playing predefined animations.

The current implementation provides two starting points:

**Expressive motion**

The waving policy uses reference motion data and robot-state observations to produce joint targets for coordinated gestures. Contributors can explore motion imitation, expressive actions, gesture transitions, and policy-driven character behavior.

**Locomotion**

The walking policy processes velocity commands and robot-state history to produce joint targets for the full humanoid. Contributors can investigate improved stability, command tracking, turning, gait transitions, and different locomotion policies.

Both implementations use ONNX Runtime for policy inference and MuJoCo for physical simulation.

New policies do not need to follow the exact architectures of the existing wave and walk controllers.

### Potential Policy Contributions

- New expressive actions and reference motion trajectories.
- Improved wave, dance, and gesture policies.
- More stable velocity-conditioned walking.
- Turning, lateral locomotion, and gait transitions.
- Autonomous balancing and fall recovery.
- Policy observation and action-space improvements.
- Alternative feedback controllers and actuator models.
- Reinforcement learning and imitation learning experiments.
- Policy evaluation and simulation-to-hardware research.

Contributors are encouraged to document policy inputs, outputs, joint ordering, runtime requirements, training assumptions, and simulation results.

## Contribute New Behaviors

Want to give Bandit a new personality?

Create a gesture, animation, reaction, motion policy, or movement sequence using the existing simulation tools.

Possible contributions include:

- Greeting and waving variations.
- Celebrations, dancing, and expressive reactions.
- New ONNX-based expressive policies.
- Head and tail movements for robot variants with suitable joints and actuators.
- Improved joint control and motion timing.
- New physics experiments.
- Interactive or sensor-driven behaviors.

Test your contribution locally, document how it works, and submit a pull request.

As the platform evolves, we aim to introduce reusable control interfaces, portable motion formats, and community motion-authoring tools to make new capabilities easier to create, validate, and share.

## Build Your Own Controllers

Developers can use the standalone robot description, MuJoCo simulation, and existing controller implementations to explore different approaches to motion control.

The repository currently includes two complementary approaches.

**Reference motion controller**

The standalone Python example demonstrates:

- Inverse kinematics to calculate arm poses from wrist-position targets.
- Joint-space feedback control to track target positions.
- Actuator-driven simulation with joint dynamics, gravity, and contacts.
- Smooth pose transitions for coordinated expressive motion.

**Policy-driven controllers**

The physical wave and walking implementations demonstrate:

- ONNX policy inference using robot observations.
- Policy outputs converted into joint-position targets.
- PD control for tracking those targets through MuJoCo actuators.
- Joint and actuator mapping validation.
- Different observation structures for expressive motion and locomotion.
- Simulation diagnostics for assessing policy behavior.

These controllers are reference implementations, not restrictions on what contributors can build.

Our longer-term goal is to support reusable controller interfaces across different robot configurations, physics environments, and eventually physical hardware.

## Simulation Integrations

Bandit is being developed for experimentation across multiple simulation environments.

The primary locally validated physics implementation currently uses **MuJoCo**.

We have also established a dedicated SimArena integration project to bring Bandit's visual model, articulated structure, and physics configuration into a browser-based simulation environment.

[**Bandit SimArena Integration**](https://github.com/banditteam/bandit-simarena-integration)

The integration repository includes scene and robot-asset configurations for SimArena, including its 24-DOF articulated robot structure.

The integration remains under development. Visual importing and articulated scene configuration have been established, while full-mesh execution in SimArena's local MuJoCo backend and integration of Bandit's ONNX policies remain ongoing work.

Contributors interested in browser-based simulation, controller integration, robotics experimentation, and physics interoperability are welcome to participate.

## Development Roadmap

Bandit is an evolving community project.

| Area | Status |
|---|---|
| Custom Bandit URDF and visual assets | Available |
| Standalone Bandit MJCF and MuJoCo assets | Available |
| MuJoCo physics simulation | Available |
| Supported standing and expressive reference controller | Available |
| Pretrained ONNX wave policy | Available |
| Pretrained ONNX locomotion policy | Available |
| Experimental free-base walking simulation | Available |
| Python controllers and motion examples | Available |
| SimArena scene and articulated robot integration | Available — ongoing development |
| Reusable Control SDK | Planned |
| Community motion authoring and behavior library | In development |
| Improved autonomous balancing and robust locomotion | Open for contributions |
| Hardware and joint extensions | Open for contributions |
| ROS 2 integration | Future development |
| Additional simulator integrations | Open for contributions |
| Sim-to-real validation and physical deployment | Future development |

Want to help build one of these capabilities? Open an Issue or start a discussion.

## The Bandit Ecosystem

Bandit is organized into four complementary open-source repositories, each focused on a different part of the robot.

### Bandit Robotics

**Simulate it. Control it. Extend it.**

The main robotics development platform, including MuJoCo simulation, motion policies, locomotion research, controllers, and physical motion experimentation.

[Bandit Robotics](https://github.com/banditteam/bandit-robotics)

### Bandit Description

**Import it. Customize it. Make it move.**

The robot description workspace, including URDF resources, geometry, materials, and joint structure.

Use it as the starting point for your own simulation environment or robot variant.

[Bandit GitHub Organization](https://github.com/banditteam)

### Bandit Design

**Redesign it. Build it. Make it yours.**

The design workspace for Blender models, custom shells, mascot development, hardware concepts, and future 3D-printable components.

[Bandit GitHub Organization](https://github.com/banditteam)

### Bandit SimArena Integration

**Bring it online. Simulate it. Experiment together.**

The browser-simulation integration project, connecting Bandit's custom appearance and articulated robot structure with SimArena.

The repository includes the scene and asset definitions being developed for browser-based robotics experimentation.

[Bandit SimArena Integration](https://github.com/banditteam/bandit-simarena-integration)

## How to Contribute

Bandit is built around open collaboration.

1. Fork the repository or create a feature branch.
2. Build a new behavior, motion policy, controller, simulation improvement, or experimental feature.
3. Test your changes locally and document how they work.
4. Submit a Pull Request.
5. Help other contributors improve and expand your work.

For contributions involving motion policies, please include relevant model files or reproducible instructions, observation and action specifications, dependencies, execution commands, and testing results.

We welcome contributions from beginners and experienced robotics developers alike.

See [CONTRIBUTING.md](CONTRIBUTING.md) for additional guidance.

## Our Vision

We believe the most interesting robots won't be developed by a single team.

They'll evolve through communities of people experimenting, creating, and contributing together.

One developer might teach Bandit to dance. Another might develop a new joint controller. Someone else might design a better mechanical component, create a new sensor integration, or contribute toward autonomous walking.

Every improvement becomes an opportunity for someone else to build further.

Bandit is open for experimentation. Its future belongs to the people building it.

**Build it. Move it. Make it yours.**

[Robot Description](https://github.com/banditteam) · [Bandit Design](https://github.com/banditteam) · [SimArena Integration](https://github.com/banditteam/bandit-simarena-integration) · [Contribute](CONTRIBUTING.md)
