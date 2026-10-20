# Bandit Robotics

Bandit Robotics is the open-source development platform for **Bandit**, a community-driven humanoid robot built to evolve through experimentation, collaboration, and shared innovation.

Start with a working robot, bring it into simulation, experiment with movements, develop new controllers, and share what you build.

Whether you're a robotics engineer, developer, researcher, student, or enthusiast, Bandit gives you a foundation to explore robotics and contribute to a growing open-source ecosystem.

**Bandit is a starting point, not a finished robot.**

## What Can You Build?

- **Expressive behaviors** — Make Bandit wave, dance, gesture, react, or develop its own personality.
- **Custom controllers** — Experiment with joint movements, control algorithms, and new robotic capabilities.
- **Physics simulations** — Explore joint dynamics, gravity, collisions, and robotic motion.
- **Locomotion** — Contribute toward balancing, walking, turning, and more advanced movement.
- **Hardware extensions** — Add joints, sensors, actuators, or new components and test their behavior in simulation.
- **Community experiments** — Build new applications, share examples, and extend what Bandit can do.

Your contribution doesn't need to be a major breakthrough. A new gesture, better controller, simulation improvement, or useful experiment can help move the project forward.

## What's Included?

The initial Bandit Robotics release provides a working simulation and development foundation.

| Component | Description |
|---|---|
| **Bandit Description** | Custom Bandit robot model, including URDF, meshes, textures, and joint definitions. |
| **MuJoCo Simulation** | Physics-based simulation using the original RoboParty RP1 dynamics as a foundation. |
| **Expressive Demo** | A working reference demonstration featuring supported standing and arm movements. |
| **Controller Examples** | Python examples for exploring and extending Bandit's motion. |
| **Simulation Tools** | Scripts and utilities used to prepare and run Bandit. |
| **Tests & Documentation** | Resources for validating the robot model and development environment. |

The initial robot uses a **24-degree-of-freedom architecture** based on the open-source RoboParty RP1 platform.

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/banditteam/bandit-robotics.git
cd bandit-robotics
```

### 2. Install Dependencies

Use Python 3.12 in a virtual environment.

```bash
python -m venv .venv
```

Activate the environment:

**Windows (PowerShell):**

```powershell
.\.venv\Scripts\Activate.ps1
```

**Linux/macOS:**

```bash
source .venv/bin/activate
```

Install the dependencies:

```bash
pip install -r requirements.txt
```

### 3. Launch Bandit

Run the existing MuJoCo reference demonstration:

```bash
python examples/run_bandit.py
```

Bandit will load into MuJoCo and perform its expressive movement sequence.

The demonstration uses temporary standing support for predictable motion. It is not an autonomous balancing or walking controller.

### 4. Explore the Robot

You can inspect the robot description, examine its joint definitions, modify controller scripts, or create new experiments using the provided examples.

For the standalone robot model, visit [Bandit Description](https://github.com/banditteam/bandit-description).

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

Test your contribution locally, document how it works, and submit a pull request.

As the platform evolves, we plan to introduce a reusable control SDK and portable behavior definitions to make sharing new capabilities easier.

## Build Your Own Controllers

Bandit is designed for experimentation.

Developers can use the current robot description and MuJoCo reference implementation to experiment with joint targets, actuator commands, physical interactions, and movement sequences.

The existing expressive demonstration is a reference implementation—not a restriction on what contributors can build.

Our longer-term goal is to provide reusable controller interfaces that support different robot configurations and simulation environments.

## Development Roadmap

Bandit is an evolving community project.

| Area | Status |
|---|---|
| Custom Bandit URDF and visual assets | Available |
| MuJoCo simulation | Available |
| Supported standing and expressive wave | Available |
| Python motion examples | Initial release |
| Reusable Control SDK | Planned |
| Community behavior library | Planned |
| Hardware and joint extensions | Open for contributions |
| Autonomous balancing and locomotion | Future development |
| ROS 2 integration | Future development |
| Additional simulator integrations | Open for contributions |

**Want to help build one of these capabilities? Open an Issue or start a discussion.**

## The Bandit Ecosystem

Bandit is organized into three open-source repositories, each focused on a different part of the robot.

### [Bandit Robotics](https://github.com/banditteam/bandit-robotics)

**Simulate it. Control it. Extend it.**

The main development platform for robotics software, simulation, expressive behaviors, and motion control.

### [Bandit Description](https://github.com/banditteam/bandit-description)

**Import it. Customize it. Make it move.**

The standalone robot description, including the URDF, geometry, materials, and joint structure.

Use it as the starting point for your own simulation environment or robot variant.

### [Bandit Design](https://github.com/banditteam/bandit-design)

**Redesign it. Build it. Make it yours.**

The mechanical and visual design workspace for Blender models, custom shells, hardware concepts, and future 3D-printable components.

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

**Bandit is open for experimentation. Its future belongs to the people building it.**

---

**Build it. Move it. Make it yours.**

[Robot Description](https://github.com/banditteam/bandit-description) · [Mechanical Design](https://github.com/banditteam/bandit-design) · [Contribute](CONTRIBUTING.md)
