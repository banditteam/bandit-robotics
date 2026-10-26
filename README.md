# Bandit Robotics

Bandit Robotics is the open-source development platform for Bandit, a community-driven humanoid robot built to evolve through experimentation, collaboration, and shared innovation.

Start with a working robot, bring it into simulation, experiment with movements, develop new controllers, and share what you build.

Whether you're a robotics engineer, developer, researcher, student, or enthusiast, Bandit gives you a foundation to explore robotics and contribute to a growing open-source ecosystem.

**Bandit is a starting point, not a finished robot.**

## What Can You Build?

- **Expressive behaviors** — Make Bandit wave, dance, gesture, react, or develop its own personality.
- **Custom controllers** — Experiment with joint movements, control algorithms, and new robotic capabilities.
- **Physics simulations** — Explore joint dynamics, gravity, collisions, inertia, and robotic motion.
- **Locomotion** — Contribute toward balancing, walking, turning, and more advanced movement.
- **Hardware extensions** — Add joints, sensors, actuators, or new components and test their behavior in simulation.
- **Community experiments** — Build new applications, share examples, and extend what Bandit can do.

Your contribution doesn't need to be a major breakthrough. A new gesture, better controller, simulation improvement, or useful experiment can help move the project forward.

## What's Included?

Bandit Robotics provides a standalone simulation and development foundation.

| Component | Description |
|---|---|
| **Bandit Description** | Custom robot model with URDF, MJCF, meshes, textures, and joint definitions. |
| **MuJoCo Simulation** | Standalone physics-based simulation using Bandit's own model and assets. |
| **Expressive Demo** | Supported standing and overhead waving, driven by joint actuators with MuJoCo physics. |
| **Controller Examples** | Python examples for inverse kinematics, joint movements, and actuator control. |
| **Simulation Tools** | Scripts and utilities used to prepare and run Bandit. |
| **Optional Compiled Model** | Precompiled MJB for faster model loading, available through Git LFS. |
| **Tests & Documentation** | Resources for validating the robot model and development environment. |


## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/banditteam/bandit-robotics.git
cd bandit-robotics
```

**The precompiled MJB is optional.** You can run Bandit directly from its editable MJCF and accompanying assets.

To avoid automatically downloading large Git LFS files when cloning, use:

```bash
GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/banditteam/bandit-robotics.git
cd bandit-robotics
```

For Windows PowerShell, use:

```powershell
$env:GIT_LFS_SKIP_SMUDGE = "1"
git clone https://github.com/banditteam/bandit-robotics.git
Remove-Item Env:\GIT_LFS_SKIP_SMUDGE
cd bandit-robotics
```

The compiled MJB is retained in the repository as an optional asset for contributors who prefer faster startup.

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

Launch the standalone simulation:

```bash
python examples/run_bandit_standalone.py
```

Bandit will load into MuJoCo and perform its expressive sequence:

1. Supported standing.
2. Both arms raising overhead.
3. Side-to-side waving.
4. Returning to the resting position.

The demonstration uses actuator-driven joint control with simulated dynamics, inertia, gravity, and contacts.

A temporary development support constraint stabilizes Bandit's base. This is **not an autonomous balancing or walking demonstration**.

**Model loading:** The launcher uses `bandit.mjb` when a valid compiled binary is available, otherwise it falls back to `bandit.xml`.

No MJB download is required to run the simulation.

### 4. Explore the Robot

Bandit's standalone MuJoCo model is located in:

```text
bandit_description/
├── mjcf/
│   ├── bandit.xml
│   └── bandit.mjb        # Optional, Git LFS
└── assets/
    ├── meshes/
    └── textures/
```

The editable MJCF is the primary robot description and source of truth.

The MJB is an optional compiled snapshot for faster loading. It is not required for editing the robot or developing controllers.

**Optional MJB download**

If you cloned without downloading LFS assets and want the compiled model later:

```bash
git lfs pull --include="bandit_description/mjcf/bandit.mjb"
```

After downloading, the standalone launcher can use the compiled version automatically.

If you modify the MJCF or its assets, regenerate the compiled binary before using it again.

For Bandit's separately maintained URDF and robot geometry, visit [Bandit Description](https://github.com/banditteam/bandit-description).

## Contribute New Behaviors

Want to give Bandit a new personality?

Create a gesture, animation, reaction, or movement sequence using the existing Python simulation tools.

Possible contributions include:

- Greeting and waving variations.
- Celebrations, dancing, and expressive reactions.
- Head and tail movements for compatible robot variants.
- Improved joint control and motion timing.
- New physics experiments.
- Interactive or sensor-driven behaviors.

The existing expressive demonstration includes inverse-kinematics-based arm positioning, actuator control, smooth transitions, and physics-based execution.

Test your contribution locally, document how it works, and submit a pull request.

As the platform evolves, we plan to introduce a reusable control SDK and portable behavior definitions to make sharing new capabilities easier.

## Build Your Own Controllers

Bandit is designed for experimentation.

Developers can use the standalone robot description and MuJoCo implementation to experiment with joint targets, actuator commands, physical interactions, and movement sequences.

The existing expressive demonstration combines:

- **Inverse kinematics** for calculating target arm poses.
- **Joint-space control** for tracking desired positions.
- **Actuator-driven physics** for dynamic simulation.
- **Smooth sequencing** for transitions between poses.

These are reference implementations, not restrictions on what contributors can build.

Our longer-term goal is to provide reusable controller interfaces supporting different robot configurations and simulation environments.

## Development Roadmap

Bandit is an evolving community project.

| Area | Status |
|---|---|
| Custom Bandit URDF and visual assets | Available |
| Standalone MuJoCo MJCF and assets | Available |
| Supported standing and physics-based expressive wave | Available |
| Python motion examples | Available |
| Optional compiled MJB | Available |
| Reusable Control SDK | Planned |
| Community behavior library | Planned |
| Head, tail, and hardware extensions | Open for contributions |
| Autonomous balancing and locomotion | In development |
| ROS 2 integration | Future development |
| Additional simulator integrations | Open for contributions |

Want to help build one of these capabilities? Open an Issue or start a discussion.

## The Bandit Ecosystem

Bandit is organized into three open-source repositories, each focused on a different part of the robot.

### Bandit Robotics

**Simulate it. Control it. Extend it.**

The main development platform for robotics software, simulation, expressive behaviors, and motion control.

[Bandit Robotics](https://github.com/banditteam/bandit-robotics)

### Bandit Description

**Import it. Customize it. Make it move.**

The standalone robot description, including the URDF, geometry, materials, and joint structure.

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
