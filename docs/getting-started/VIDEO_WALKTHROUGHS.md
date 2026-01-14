# Video Walkthroughs

This page provides video tutorials and walkthroughs for the ripple-env development environment. Videos are organized by topic and skill level.

---

## Quick Start Videos

### Environment Setup

| Topic | Duration | Level | Link |
|-------|----------|-------|------|
| Installing Nix on Linux | ~5 min | Beginner | [Placeholder - Coming Soon](#contributing-videos) |
| Installing Nix on macOS | ~5 min | Beginner | [Placeholder - Coming Soon](#contributing-videos) |
| Windows WSL2 Bootstrap | ~10 min | Beginner | [Placeholder - Coming Soon](#contributing-videos) |
| First `nix develop` Session | ~3 min | Beginner | [Placeholder - Coming Soon](#contributing-videos) |

### ROS2 Basics

| Topic | Duration | Level | Link |
|-------|----------|-------|------|
| Creating Your First ROS2 Package | ~8 min | Beginner | [Placeholder - Coming Soon](#contributing-videos) |
| Building with Colcon | ~5 min | Beginner | [Placeholder - Coming Soon](#contributing-videos) |
| Publishing and Subscribing | ~10 min | Beginner | [Placeholder - Coming Soon](#contributing-videos) |
| Using ROS2 Launch Files | ~8 min | Intermediate | [Placeholder - Coming Soon](#contributing-videos) |

---

## Deep Dive Videos

### Nix and Flakes

| Topic | Duration | Level | Link |
|-------|----------|-------|------|
| Understanding flake.nix | ~15 min | Intermediate | [Placeholder - Coming Soon](#contributing-videos) |
| Adding Packages to the Flake | ~10 min | Intermediate | [Placeholder - Coming Soon](#contributing-videos) |
| Creating Custom DevShells | ~12 min | Advanced | [Placeholder - Coming Soon](#contributing-videos) |
| Flake Modularization | ~20 min | Advanced | [Placeholder - Coming Soon](#contributing-videos) |

### Docker Services

| Topic | Duration | Level | Link |
|-------|----------|-------|------|
| Docker Compose Overview | ~8 min | Beginner | [Placeholder - Coming Soon](#contributing-videos) |
| Starting Observability Stack | ~10 min | Intermediate | [Placeholder - Coming Soon](#contributing-videos) |
| LocalAI Setup and Usage | ~15 min | Intermediate | [Placeholder - Coming Soon](#contributing-videos) |
| Custom Docker Configurations | ~12 min | Advanced | [Placeholder - Coming Soon](#contributing-videos) |

### Agent Configuration

| Topic | Duration | Level | Link |
|-------|----------|-------|------|
| ARIA Orchestrator Overview | ~10 min | Intermediate | [Placeholder - Coming Soon](#contributing-videos) |
| Configuring AI Agents | ~15 min | Intermediate | [Placeholder - Coming Soon](#contributing-videos) |
| Multi-Agent Workflows | ~20 min | Advanced | [Placeholder - Coming Soon](#contributing-videos) |

---

## External Video Resources

### Nix Tutorials

These external resources provide excellent Nix education:

| Resource | Description | Link |
|----------|-------------|------|
| Zero to Nix | Official beginner-friendly Nix tutorial | [zero-to-nix.com](https://zero-to-nix.com/) |
| Nix Pills | Deep dive into Nix internals | [Nix Pills](https://nixos.org/guides/nix-pills/) |
| Tweag Nix Videos | Professional Nix tutorials | [Tweag YouTube](https://www.youtube.com/@tweaboratories) |
| Jon Ringer's Channel | Practical NixOS tutorials | [YouTube](https://www.youtube.com/@jonringer117) |

### ROS2 Tutorials

| Resource | Description | Link |
|----------|-------------|------|
| ROS2 Official Tutorials | Comprehensive ROS2 learning | [docs.ros.org](https://docs.ros.org/en/humble/Tutorials.html) |
| The Construct | ROS2 online courses | [theconstructsim.com](https://www.theconstructsim.com/) |
| Articulated Robotics | Practical ROS2 projects | [YouTube](https://www.youtube.com/@ArticulatedRobotics) |
| Robotics Back-End | ROS2 programming tutorials | [YouTube](https://www.youtube.com/@RoboticsBackEnd) |

### Docker and DevOps

| Resource | Description | Link |
|----------|-------------|------|
| Docker Official | Docker fundamentals | [docker.com/get-started](https://www.docker.com/get-started/) |
| TechWorld with Nana | DevOps and Docker tutorials | [YouTube](https://www.youtube.com/@TechWorldwithNana) |

---

## Recommended Learning Paths

### Path 1: New to Everything

1. Watch: [Zero to Nix](https://zero-to-nix.com/) (external)
2. Follow: [Interactive Onboarding Tutorial](./ONBOARDING_TUTORIAL.md)
3. Watch: Environment Setup videos (when available)
4. Practice: [Progressive Examples - Beginner](./PROGRESSIVE_EXAMPLES.md#beginner-level)

### Path 2: Know ROS2, New to Nix

1. Watch: [Zero to Nix](https://zero-to-nix.com/) (external)
2. Read: [Flake Modularization](../NIX_FLAKE_MODULARIZATION.md)
3. Watch: Nix and Flakes videos (when available)
4. Practice: Customizing the flake

### Path 3: Know Nix, New to ROS2

1. Watch: [ROS2 Official Tutorials](https://docs.ros.org/en/humble/Tutorials.html) (external)
2. Follow: [Progressive Examples](./PROGRESSIVE_EXAMPLES.md)
3. Practice: Building ROS2 packages in the Nix environment

### Path 4: Advanced User

1. Watch: Agent Configuration videos (when available)
2. Read: [ARIA Orchestrator](../audits/MANUS_ARIA_ORCHESTRATOR.md)
3. Explore: [Edge Deployment](../edge-service/gateway/EDGE_DEPLOYMENT.md)
4. Contribute: Record videos for others!

---

## Contributing Videos

We welcome video contributions! Here's how to help:

### Recording Guidelines

1. **Keep it focused**: One topic per video, 5-15 minutes ideal
2. **Show the terminal**: Use a clear, readable font (14pt+)
3. **Explain as you go**: Narrate your actions
4. **Include timestamps**: Add chapter markers for longer videos
5. **Test your examples**: Ensure all commands work

### Recommended Tools

| Tool | Platform | Notes |
|------|----------|-------|
| OBS Studio | All | Free, professional quality |
| Asciinema | Terminal only | Perfect for CLI demos |
| ScreenPal | All | Easy editing |
| Loom | All | Quick sharing |

### Video Hosting

Options for hosting contributed videos:
- YouTube (recommended for discoverability)
- Asciinema.org (for terminal recordings)
- Project wiki (for short clips)

### Submission Process

1. Record your video following the guidelines
2. Upload to your preferred platform
3. Open a PR updating this file with:
   - Video title and description
   - Duration
   - Skill level
   - Link

### Video Ideas Wanted

We especially need videos for:

- [ ] Windows WSL2 complete setup
- [ ] macOS with Apple Silicon setup
- [ ] Debugging common Nix errors
- [ ] ROS2 simulation setup
- [ ] LocalAI model configuration
- [ ] Grafana dashboard creation
- [ ] CI/CD pipeline walkthrough

---

## Asciinema Recordings

Terminal-only demonstrations using [asciinema](https://asciinema.org/):

### Environment

| Demo | Description | Link |
|------|-------------|------|
| First nix develop | Entering the shell for first time | [Placeholder](#contributing-videos) |
| Shell switching | Switching between dev shells | [Placeholder](#contributing-videos) |
| Package addition | Adding packages with Pixi | [Placeholder](#contributing-videos) |

### Building

| Demo | Description | Link |
|------|-------------|------|
| Colcon build | Full build workflow | [Placeholder](#contributing-videos) |
| Incremental build | Rebuilding single package | [Placeholder](#contributing-videos) |
| Test execution | Running and viewing tests | [Placeholder](#contributing-videos) |

---

## Live Streams and Recordings

Past live coding sessions and community events:

| Event | Date | Description | Link |
|-------|------|-------------|------|
| *None yet* | - | Community contributions welcome! | [Contribute](#contributing-videos) |

---

## FAQ

### Q: Why are many videos marked "Coming Soon"?

This is a new documentation initiative. We're building out video content progressively. External resources are available now for core topics, and we welcome community contributions.

### Q: Can I use these videos in my own tutorials?

Videos created by the project are available under the same license as the project. External resource links are provided for convenience - check their individual licenses.

### Q: How can I request a specific video topic?

Open a [GitHub Discussion](https://github.com/FlexNetOS/ripple-env/discussions) with the "documentation" tag describing what you'd like to see covered.

---

## See Also

- [Interactive Onboarding Tutorial](./ONBOARDING_TUTORIAL.md) - Step-by-step text guide
- [Progressive Examples](./PROGRESSIVE_EXAMPLES.md) - Code examples by skill level
- [Getting Started](./GETTING_STARTED.md) - Quick setup reference
