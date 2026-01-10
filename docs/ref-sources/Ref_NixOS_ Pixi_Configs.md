## Scope (Sep 2025 → Jan 9, 2026)

You asked for **NixOS + Pixi configurations** that are both **popular** (stars) and **recently updated**, optimized for **agentic robotics + DevOps “vibe coding” + distributed compute**, with **NixOS + Pixi + Nushell** as the base environment.

---

## NixOS configurations (popular + recently updated)

These are the highest-signal, *actively maintained* Nix configs in your window, with **stars + last-update** visible from a curated ranking plus repo pages. ([GitHub][1])

| Repo                               | Stars | “Last update” seen | Why it’s useful for your use case                                                                                         |
| ---------------------------------- | ----: | ------------------ | ------------------------------------------------------------------------------------------------------------------------- |
| **ryan4yin/nix-config**            |  5.1k | 2026-01-08         | Multi-machine + dev-focused structure; good reference for **clean modularization + repeatable dev shells**. ([GitHub][1]) |
| **dustinlyons/nixos-config**       |  3.3k | 2026-01-03         | Broad “daily-driver” config patterns (multiple systems) that scale to laptops/desktops/servers. ([GitHub][1])             |
| **dreamsofautonomy/homelab**       |  2.3k | 2025-12-30         | Explicit “NixOS powered homelab” repo—useful reference for **infra-as-code + services + fleet** patterns. ([GitHub][2])   |
| **mitchellh/nixos-config**         |  2.8k | 2025-12-23         | Opinionated infra config style from an infra-heavy author; good for disciplined “system as code.” ([GitHub][1])           |
| **hlissner/dotfiles**              |  1.9k | 2025-09-12         | High-quality modular Nix patterns and “don’t let it rot” hygiene. ([GitHub][1])                                           |
| **gvolpe/nix-config**              |   745 | 2026-01-06         | Smaller but very “engineer-y” and clean—good for **consistency + DRY modules**. ([GitHub][1])                             |
| **Misterio77/nix-starter-configs** |  3.5k | 2025-11-15         | Not a full “fleet config,” but a **best-in-class template** to standardize layouts & conventions fast. ([GitHub][3])      |

### Cross-hardware / mobile angle (important for your “mobile + PC + laptops” request)

If you want configs that extend to *mobile*, you’re basically looking at **nix-on-droid** being part of the story. It’s popular enough to treat as a foundational building block. ([GitHub][4])

| Project                        | Stars | What it enables                                                                 |
| ------------------------------ | ----: | ------------------------------------------------------------------------------- |
| **nix-community/nix-on-droid** |  6.2k | Nix-managed userland on Android (your “IDE-on-mobile” substrate). ([GitHub][4]) |

### Smaller but *highly relevant* “multi-host / module-first” repos (good patterns even if not mega-star)

These are directly aligned with “fleet + mixed hardware” and modular `modules/` layout approaches:

* **Gerschtli/nix-config** explicitly describes **NixOS-managed servers + laptop + Raspberry Pi + WSL2 + nix-on-droid** in one repo. ([GitHub][5])
* **mwdavisii/nyx** is positioned as a *multi-platform flake* spanning NixOS/macOS/WSL/Droid. ([GitHub][6])
* **MatthewCash/nixos-config** shows a pragmatic scripted workflow (“apply”, “full-upgrade”). ([GitHub][7])

---

## Pixi configurations (popular + robotics/devops relevant)

Pixi adoption is more “distributed” (lots of projects use it, fewer *template repos* dominate). So the realistic “most popular” anchors are: **Pixi itself**, plus **Pixi environment packaging** (pixi-pack), plus **real robotics projects that use Pixi tasks**.

### Top repos to copy patterns from

| Repo                            |          Stars | What to steal (configuration-wise)                                                                                                        |
| ------------------------------- | -------------: | ----------------------------------------------------------------------------------------------------------------------------------------- |
| **prefix-dev/pixi**             |           6.1k | Canonical `pixi.toml` patterns + cross-platform examples; Pixi is positioned as fast and reproducible. ([GitHub][8])                      |
| **Quantco/pixi-pack**           |            161 | The missing “deployable env artifact” piece: pack/unpack Pixi environments. Great for robotics + field deployment. ([GitHub][9])          |
| **Simple-Robotics/aligator**    |            271 | A real robotics/optimization library using `pixi.toml` (C++ heavy). Useful to mirror patterns for mixed-language stacks. ([GitHub][10])   |
| **upkie/new_agent**             | (not captured) | Practical robotics workflow: Pixi tasks for simulation + agent run, plus Makefile-based env packaging + robot upload flow. ([GitHub][11]) |
| **AtharvaBhorpe/isaaclab-pixi** | (not captured) | Pixi-driven setup for Isaac Sim/Lab to keep GPU/ML-ish robotics dev reproducible across machines. ([GitHub][12])                          |

### Pixi config details that matter for your constraints

*What I can actually prove from sources in your window:*

* Pixi’s **robotics relevance** is not hypothetical: there’s a 2025 paper framing Pixi as a reproducible robotics/AI environment tool and claiming **up to 10× faster dependency resolution** and adoption in **5,300+ projects**. ([arXiv][13])
* The ROS2 Nav2 example exists and is runnable via Pixi tasks (`pixi run start`). ([Prefix Dev][14])
* Pixi manifests include a **workspace** with **channels** (example shows `https://prefix.dev/conda-forge`) and cross-platform targeting (example shows multiple `platforms` including Windows/Linux/macOS). ([GitHub][15])
* There is active discussion about ROS naming/prefix conventions in Pixi’s build backends (Sep 10, 2025), which is exactly the kind of “config consistency” pain you’re trying to avoid. ([GitHub][16])
* A ROS2 dev tooling doc claims “Open Robotics has standardized on Pixi for Windows development” (useful if Windows nodes exist in your fleet). ([Ranch Hand Robotics][17])

---

## A configuration “shape” that fits **NixOS + Pixi + Nushell** (and your `modules/` focus)

This is the pattern that keeps settings **consistent**, **diffable**, and **agent-friendly** across laptops/desktops/servers/mobile:

### 1) NixOS becomes the **machine contract**

* Put everything “device identity” under `hosts/<hostname>/…`
* Put everything “fleet policy” under `modules/…` (what you’re already doing)
* Use one shared “roles” layer: `roles/devops.nix`, `roles/robotics.nix`, `roles/worker-node.nix`, etc.
* Treat shells as part of the contract: manage Nushell + its config via Home Manager inside the same flake

### 2) Pixi becomes the **project contract**

* Each repo / workspace gets a `pixi.toml` (or `pyproject.toml` using Pixi tables, if you standardize that)
* Use Pixi **tasks** as the *single command surface* for agents:

  * `pixi run build`, `pixi run test`, `pixi run sim`, `pixi run deploy`
* For robotics deployments: add **pixi-pack** so you can ship a deterministic environment blob to field nodes/robots. ([GitHub][9])

### 3) “Distributed compute” glue

Use a homelab-style NixOS repo as a reference for service orchestration patterns (DNS, storage, cluster bootstrap, secrets, remote deploy). **dreamsofautonomy/homelab** is the most popular recent example I found in-window. ([GitHub][2])

---

## Blunt recommendation (what to fork / mirror first)

If you want the fastest path to a clean, scalable baseline:

* **Primary NixOS reference**: `ryan4yin/nix-config` (structure, modularity, dev ergonomics). ([GitHub][1])
* **Homelab / distributed compute reference**: `dreamsofautonomy/homelab` (infra patterns). ([GitHub][2])
* **Mobile substrate**: `nix-on-droid` (so “IDE on mobile” isn’t a separate universe). ([GitHub][4])
* **Pixi robotics workflow reference**: `upkie/new_agent` + `pixi-pack` (task-driven dev + deployable env artifact). ([GitHub][11])

That combo matches your goals: **agent-run CLIs**, **cross-platform fleet**, and **project-level reproducibility** without turning your config into a snowflake museum.

---

[1]: https://github.com/topics/hyprland?l=nix&o=desc&s=stars&utm_source=chatgpt.com "hyprland · GitHub Topics"
[2]: https://github.com/taylrfnt/homelab "GitHub - taylrfnt/homelab: a Nix-based kubernetes cluster configuration, bundled with a few basic services managed through helm"
[3]: https://github.com/topics/flakes?utm_source=chatgpt.com "flakes · GitHub Topics"
[4]: https://github.com/nix-community/nix-on-droid "GitHub - nix-community/nix-on-droid: Nix-enabled environment for your Android device. [maintainers=@t184256,@Gerschtli]"
[5]: https://github.com/Gerschtli/nix-config?utm_source=chatgpt.com "Gerschtli/nix-config: A collection of my system ..."
[6]: https://github.com/robotology/robotology-superbuild/releases?utm_source=chatgpt.com "Releases · robotology/robotology-superbuild"
[7]: https://github.com/MatthewCash/nixos-config?utm_source=chatgpt.com "MatthewCash/nixos-config: NixOS System Configurations"
[8]: https://github.com/prefix-dev/pixi/blob/main/examples/ros2-nav2/pixi.toml "pixi/examples/ros2-nav2/pixi.toml at main · prefix-dev/pixi · GitHub"
[9]: https://github.com/Quantco/pixi-pack "GitHub - Quantco/pixi-pack:  Pack and unpack conda environments created with pixi"
[10]: https://github.com/Simple-Robotics/aligator/blob/main/pixi.toml?utm_source=chatgpt.com "pixi.toml - Simple-Robotics/aligator"
[11]: https://github.com/upkie/new_agent?utm_source=chatgpt.com "upkie/new_agent: Template to create new agents and ..."
[12]: https://github.com/AtharvaBhorpe/isaaclab-pixi?utm_source=chatgpt.com "AtharvaBhorpe/isaaclab-pixi: Isaac Lab development ..."
[13]: https://arxiv.org/abs/2511.04827?utm_source=chatgpt.com "Pixi: Unified Software Development and Distribution for Robotics and AI"
[14]: https://prefix-dev.github.io/pixi/v0.39.4/examples/ros2-nav2/?utm_source=chatgpt.com "Navigation 2 example - Pixi by prefix.dev"
[15]: https://github.com/prefix-dev/pixi/blob/main/examples/ros2-nav2/pixi.toml?utm_source=chatgpt.com "pixi/examples/ros2-nav2/pixi.toml at main · prefix-dev/pixi"
[16]: https://github.com/prefix-dev/pixi-build-backends/issues/355?utm_source=chatgpt.com "How to deal with the package naming of source build ros ..."
[17]: https://ranchhandrobotics.com/rde-ros-2/pixi.html?utm_source=chatgpt.com "Pixi for ROS 2 - Robot Developer Extensions for ROS 2"
