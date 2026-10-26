# Bandit Robotics

Bandit Robotics is the open-source development platform for Bandit, a community-driven humanoid robot built to evolve through experimentation, collaboration, and shared innovation.

Start with a working robot, bring it into simulation, experiment with movements, develop new controllers, and share what you build.

Whether you're a robotics engineer, developer, researcher, student, or enthusiast, Bandit gives you a foundation to explore robotics and contribute to a growing open-source ecosystem.

**Bandit is a starting point, not a finished robot.**

## What Can You Build?

- **Expressive behaviors** — Make Bandit wave, dance, gesture, react, or develop its own personality.
- **Custom controllers** — Experiment with joint movements, control algorithms, and new robotic capabilities.
- **Physics simulations** — Explore joint dynamics, gravity, collisions, inertia, and robotic motion using MuJoCo.
- **Locomotion** — Contribute toward balancing, walking, turning, and more advanced movement.
- **Hardware extensions** — Add joints, sensors, actuators, or new components and test their behavior in simulation.
- **Community experiments** — Build new applications, share examples, and extend what Bandit can do.

Your contribution doesn't need to be a major breakthrough. A new gesture, better controller, simulation improvement, or useful experiment can help move the project forward.

## What's Included?

Bandit Robotics provides a standalone simulation and development foundation built around Bandit's own robot description and assets.

| Component | Description |
|---|---|
| **Bandit Description** | Custom Bandit robot model with URDF, MJCF, meshes, textures, materials, and joint definitions. |
| **Standalone MuJoCo Model** | Independently loadable `bandit.xml` containing the 24-DoF robot structure, collision geometry, and physics configuration. |
| **Precompiled MuJoCo Model** | Optional `bandit.mjb` for significantly faster model loading without recompiling the MJCF at startup. |
| **Physics-Based Expressive Demo** | A working demonstration of supported standing, coordinated arm raising, and overhead waving using actuator-driven joint physics. |
| **Controller Examples** | Python implementations demonstrating inverse kinematics, actuator commands, pose transitions, and motion control. |
| **Simulation Tools** | Scripts and utilities used to prepare, compile, and experiment with Bandit. |
| **Tests & Documentation** | Resources for validating robot models and the development environment. |

Bandit currently uses a **24-degree-of-freedom humanoid architecture** derived from the open-source RoboParty RP1 platform.

Its underlying kinematics and physical parameters retain that foundation, but the standalone MuJoCo description, associated assets, and current expressive controller can operate without requiring the `RP1_description` directory at runtime.

The simulator supports both physics-based control and experimentation with scripted motion. Physically balanced locomotion remains an area of active development.

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/banditteam/bandit-robotics.git
cd bandit-robotics
```

Bandit includes a precompiled MuJoCo model distributed through Git LFS.

If you want the precompiled model, ensure Git LFS is installed and run:

```bash
git lfs install
git lfs pull
```

The editable MJCF remains available even without the precompiled binary.

### 2. Install Dependencies

Use Python 3.12 in a virtual environment.

```bash
python -m venv .venv
```

Activate the environment.

**Windows (PowerShell):**

```powershell
.\.venv\Scripts\Activate.ps1
```

**Linux/macOS:**

```bash
source .venv/bin/activate
```

Install dependencies:

```bash
pip install -r requirements.txt
```

### 3. Launch Bandit

Run the standalone MuJoCo expressive demonstration:

```bash
python examples/run_bandit_standalone.py
```

Bandit will load into MuJoCo and perform its reference expressive sequence:

1. Supported standing with both arms lowered.
2. Coordinated arm raising.
3. Overhead waving.
4. Arms returning to a resting position.

The demonstration uses the robot's joint actuators and MuJoCo physics engine, preserving joint dynamics, inertia, gravity, and contact behavior.

A temporary development support constraint maintains the robot's base during the sequence. **This is not an autonomous balancing or walking demonstration.**

**Fast loading:** The launcher uses `bandit_description/mjcf/bandit.mjb` when available. Otherwise, it compiles the editable `bandit.xml` automatically.

### 4. Explore the Robot

Bandit's standalone simulation files are located in:

```text
bandit_description/
├── mjcf/
│   ├── bandit.xml
│   └── bandit.mjb
└── assets/
    ├── meshes/
    └── textures/
```

The editable MJCF defines Bandit's body structure, joints, actuators, physical properties, and visual assets.

The compiled MJB provides a faster-loading representation of the same model.

You can inspect joint definitions, explore the actuator interfaces, modify controllers, or develop new simulation experiments.

For the independently maintained URDF and robot geometry resources, visit [Bandit Description](https://github.com/banditteam/bandit-description).

**Note:** When modifying the MJCF or its referenced assets, regenerate the MJB to reflect those changes. The compiled binary is a snapshot, not a live link to the source description.

## Contribute New Behaviors

Want to give Bandit a new personality?

Create a gesture, animation, reaction, or movement sequence using the existing Python simulation tools.

Possible contributions include:

- Greeting and waving variations.
- Celebrations, dancing, and expressive reactions.
- Head and tail movements for robot variants with the appropriate joints and actuators.
- Improved joint control and motion timing.
- New physics experiments.
- Interactive or sensor-driven behaviors.

The existing expressive demonstration provides a useful starting point. It already includes inverse-kinematics-based pose generation, actuator commands, smooth motion transitions, and physics-based execution.

Contributors can build on these techniques or explore alternative approaches.

Test your contribution locally, document how it works, and submit a pull request.

As the platform evolves, we plan to introduce a reusable control SDK and portable behavior definitions to make sharing new capabilities easier.

## Build Your Own Controllers

Bandit is designed for experimentation.

Developers can use the standalone robot description and MuJoCo implementation to experiment with joint targets, actuator commands, physical interactions, and movement sequences.

The existing expressive demonstration combines:

- **Inverse kinematics** to calculate target arm poses from the robot's geometry.
- **Joint-space control** to track desired positions.
- **Actuator-driven physics** to simulate motion through MuJoCo.
- **Smooth motion sequencing** to transition between expressive poses.

These components are a reference implementation, not a restriction on what contributors can build.

Developers are also free to explore kinematic animation, custom motion generation, reinforcement learning, and alternative control systems.

Our longer-term goal is to provide reusable controller interfaces that support different robot configurations and simulation environments without requiring contributors to rewrite behaviors for every backend.

## Development Roadmap

Bandit is an evolving community project.

| Area | Status |
|---|---|
| Custom Bandit URDF and visual assets | Available |
| Standalone Bandit MJCF and assets | Available |
| Precompiled MuJoCo model (MJB) | Available |
| MuJoCo physics simulation | Available |
| Supported standing and physics-based expressive wave | Available |
| Python motion and controller examples | Available |
| Reusable Control SDK | Planned |
| Community behavior library | Planned |
| Expressive behaviors and gesture expansion | Open for contributions |
| Head, tail, and hardware joint extensions | Open for contributions |
| Autonomous balancing and physics-based locomotion | In development |
| ROS 2 integration | Future development |
| Additional simulator integrations | Open for contributions |

Want to help build one of these capabilities? Open an Issue or start a discussion.

## The Bandit Ecosystem

Bandit is organized into three open-source repositories, each focused on a different part of the robot.

### Bandit Robotics

**Simulate it. Control it. Extend it.**

The main development platform for robotics software, MuJoCo simulation, expressive behaviors, physics experiments, and motion control.

Includes Bandit's standalone MuJoCo model and reference controller implementations.

[Bandit Robotics](https://github.com/banditteam/bandit-robotics)

### Bandit Description

**Import it. Customize it. Make it move.**

The robot description repository, containing Bandit's URDF, geometry, materials, and joint structure.

Use it as the starting point for your own simulation environment or robot variant.

[Bandit Description](https://github.com/banditteam/bandit-description)

### Bandit Design

**Redesign it. Build it. Make it yours.**

The mechanical and visual design workspace for Blender models, custom shells, hardware concepts, and future 3D-printable components.

[Bandit Design](https://github.com/banditteam/bandit-design)

## How to Contribute

Bandit is built around open collaboration.

1. Fork the repository or create a feature branch.
2. Build a new behavior, controller, simulation improvement, or experimental feature.
3. Test your changes and document how they work.
4. Submit a Pull Request.
5. Help other contributors improve and expand your work.

We welcome contributions from beginners and experienced robotics developers alike.

See [CONTRIBUTING.md](CONTRIBUTING.md) for additional guidance.

## Our Vision

We believe the most interesting robots won't be developed by a single team.

They'll evolve through communities of people experimenting, creating, and contributing together.

One developer might teach Bandit to dance. Another might develop a new joint controller. Someone else might design a better mechanical component, create a new sensor integration, or contribute toward autonomous walking.

Every improvement becomes an opportunity for someone else to build further.

Bandit is open for experimentation. Its future belongs to the people building it.

**Build it. Move it. Make it yours.**

[Robot Description](https://github.com/banditteam/bandit-description) · [Mechanical Design](https://github.com/banditteam/bandit-design) · [Contribute](CONTRIBUTING.md)
