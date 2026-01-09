# Copilot instructions for ros2-humble-env

## Repo goals
- Provide a reproducible ROS2 Humble dev environment using **Nix flakes** + **pixi**.
- Keep CI reliable across Linux/macOS, and keep Windows/WSL steps best-effort where kernel features vary.

## Performance expectations
This repo can feel slow in fresh environments because Nix/pixi must download/cache tools the first time.
Prefer changes that keep the **default** workflow fast:
- Use `devShells.default` for the minimal day-to-day shell.
- Use `devShells.full` only when you need extra tooling.
- In CI, prefer `nix develop .#ci --command …` where possible.

## Conventions
- `nom` (nix-output-monitor) is optional; never alias/replace the `nix` CLI.
- Avoid adding heavy tools to the default shell unless they are essential for core ROS2 development.
- Keep `shellHook` lightweight and avoid initializing prompts (starship) or direnv hooks inside the shellHook.

## Suggested fast commands
- Enter minimal environment: `nix develop`
- Enter full environment: `nix develop .#full`
- Fast flake validation: `nix flake check --no-build`

## Editing guidance
- Keep changes small and targeted.
- Update docs/tests when behavior changes.
- Avoid changing unrelated formatting.
