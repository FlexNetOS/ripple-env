# Configuration Tool Enforcement Policies
# Hard guardrails for Nix + Pixi + direnv + Home-manager architecture
# Use with: conftest test --policy .claude/policies/

package configuration

import future.keywords.in

#############################################
# RULE: No mise/rtx/.tool-versions
# Rationale: ADR-003 - Nix + Pixi provides superior reproducibility
#############################################

deny[msg] {
    input.file == ".tool-versions"
    msg := "VIOLATION: .tool-versions detected. Use Nix flake.lock + pixi.lock instead (ADR-003)"
}

deny[msg] {
    input.file == ".mise.toml"
    msg := "VIOLATION: .mise.toml detected. Use flake.nix + pixi.toml instead (ADR-003)"
}

deny[msg] {
    input.file == ".rtx.toml"
    msg := "VIOLATION: .rtx.toml detected (legacy mise). Use Nix + Pixi (ADR-003)"
}

#############################################
# RULE: No 0install feeds
# Rationale: ADR-003 - Conflicts with Nix store architecture
#############################################

deny[msg] {
    endswith(input.file, ".0install.xml")
    msg := "VIOLATION: 0install feed detected. Use Nix + Pixi (ADR-003)"
}

#############################################
# RULE: No asdf
# Rationale: mise supersedes asdf; Nix supersedes both
#############################################

deny[msg] {
    input.file == ".asdfrc"
    msg := "VIOLATION: .asdfrc detected. Use Nix flakes (ADR-003)"
}

deny[msg] {
    input.file == ".asdf"
    msg := "VIOLATION: .asdf directory detected. Use Nix flakes (ADR-003)"
}

#############################################
# RULE: Enforce direnv with use flake
# Rationale: Automatic environment activation
#############################################

deny[msg] {
    input.file == ".envrc"
    not contains(input.content, "use flake")
    msg := "VIOLATION: .envrc must contain 'use flake' for Nix integration"
}

warn[msg] {
    input.file == ".envrc"
    not contains(input.content, "use_flake_pixi")
    not contains(input.content, "pixi shell-hook")
    contains(input.content, "pixi")
    msg := "WARNING: .envrc uses pixi but may not activate it properly"
}

#############################################
# RULE: flake.nix structure requirements
# Rationale: Consistent flake-parts + devshell pattern
#############################################

deny[msg] {
    input.file == "flake.nix"
    not contains(input.content, "flake-parts")
    msg := "VIOLATION: flake.nix must use flake-parts for consistency"
}

deny[msg] {
    input.file == "flake.nix"
    not contains(input.content, "devshell")
    msg := "VIOLATION: flake.nix must use devshell for development shells"
}

warn[msg] {
    input.file == "flake.nix"
    not contains(input.content, "nix-systems/default")
    msg := "WARNING: flake.nix should use nix-systems/default for multi-platform"
}

#############################################
# RULE: pixi.toml channel requirements
# Rationale: RoboStack for ROS2 + conda-forge for Python
#############################################

deny[msg] {
    input.file == "pixi.toml"
    contains(input.content, "ros-humble")
    not contains(input.content, "robostack-humble")
    msg := "VIOLATION: ROS2 Humble packages require robostack-humble channel"
}

deny[msg] {
    input.file == "pixi.toml"
    contains(input.content, "ros-jazzy")
    not contains(input.content, "robostack-jazzy")
    msg := "VIOLATION: ROS2 Jazzy packages require robostack-jazzy channel"
}

#############################################
# RULE: Lock file requirements
# Rationale: Reproducibility requires lock files
#############################################

deny[msg] {
    input.file == "flake.nix"
    not input.sibling_files["flake.lock"]
    msg := "VIOLATION: flake.nix requires flake.lock for reproducibility"
}

deny[msg] {
    input.file == "pixi.toml"
    not input.sibling_files["pixi.lock"]
    msg := "VIOLATION: pixi.toml requires pixi.lock for reproducibility"
}

#############################################
# RULE: Home-manager module structure
# Rationale: Consistent module pattern
#############################################

deny[msg] {
    startswith(input.file, "modules/")
    endswith(input.file, ".nix")
    not contains(input.content, "lib")
    not contains(input.content, "mkDefault")
    not contains(input.content, "mkOption")
    msg := sprintf("WARNING: Module %s should use lib.mkDefault or lib.mkOption patterns", [input.file])
}

warn[msg] {
    startswith(input.file, "modules/common/")
    endswith(input.file, ".nix")
    contains(input.content, "stdenv.isLinux")
    msg := sprintf("WARNING: Common module %s has Linux-specific code. Move to modules/linux/", [input.file])
}

warn[msg] {
    startswith(input.file, "modules/common/")
    endswith(input.file, ".nix")
    contains(input.content, "stdenv.isDarwin")
    msg := sprintf("WARNING: Common module %s has macOS-specific code. Move to modules/macos/", [input.file])
}

#############################################
# RULE: DevContainer requirements
# Rationale: DevPod/Codespaces compatibility
#############################################

deny[msg] {
    input.file == ".devcontainer/devcontainer.json"
    not contains(input.content, "nix")
    msg := "VIOLATION: devcontainer.json must include Nix feature for consistency"
}

warn[msg] {
    input.file == ".devcontainer/devcontainer.json"
    not contains(input.content, "/nix")
    msg := "WARNING: devcontainer.json should mount /nix volume for caching"
}

#############################################
# RULE: Responsibility boundaries
# Rationale: Clear separation of concerns
#############################################

# System packages belong in Nix
warn[msg] {
    input.file == "pixi.toml"
    pkg := input.packages[_]
    pkg in ["git", "helix", "ripgrep", "fd", "bat", "eza", "jq", "yq"]
    msg := sprintf("WARNING: System tool '%s' in pixi.toml should be in flake.nix", [pkg])
}

# ROS2 packages belong in Pixi
warn[msg] {
    input.file == "flake.nix"
    contains(input.content, "ros-humble")
    msg := "WARNING: ROS2 packages in flake.nix should be in pixi.toml via RoboStack"
}

# Python scientific packages belong in Pixi
warn[msg] {
    input.file == "flake.nix"
    pkg := input.packages[_]
    pkg in ["numpy", "scipy", "pandas", "pytorch", "tensorflow"]
    msg := sprintf("WARNING: Python package '%s' should be in pixi.toml", [pkg])
}

#############################################
# RULE: No symlinks in repository
# Rationale: Symlinks cause issues with Git, Windows, and some tools
#############################################

deny[msg] {
    input.file_type == "symlink"
    not startswith(input.file, ".pixi/")
    not startswith(input.file, ".git/")
    msg := sprintf("VIOLATION: Symlink detected at '%s'. Use actual files or update references.", [input.file])
}

warn[msg] {
    input.tool == "home-manager"
    msg := "NOTE: Home-manager uses symlinks by design. Consider chezmoi if symlinks are unacceptable."
}

#############################################
# CHEZMOI + MISE EVALUATION TRIGGERS
# If these conditions are met, consider alternatives
#############################################

# Trigger: Too many .envrc.local overrides
warn[msg] {
    input.file == ".envrc.local"
    count(split(input.content, "\n")) > 20
    msg := "EVALUATION: Large .envrc.local may indicate need for chezmoi templating"
}

# Trigger: Multiple Python version requirements
warn[msg] {
    input.file == "pixi.toml"
    count(regex.find_n("python.*=.*\\d+\\.\\d+", input.content, -1)) > 1
    msg := "EVALUATION: Multiple Python versions may warrant mise for version switching"
}

# Trigger: Non-Nix users struggling
warn[msg] {
    input.file == "CONTRIBUTING.md"
    contains(input.content, "without Nix")
    msg := "EVALUATION: Non-Nix onboarding needs may warrant mise as fallback"
}

#############################################
# WHEN TO CONSIDER CHEZMOI + MISE
#############################################

# Document the decision criteria
metadata := {
    "current_stack": "Nix + Pixi + direnv + Home-manager + DevPod",
    "alternatives": {
        "chezmoi": {
            "consider_when": [
                "Users need portable dotfiles across non-Nix systems",
                "Template-heavy per-user configuration needed",
                "1Password/pass secret integration required"
            ],
            "do_not_use_for": [
                "Project configuration (use flake.nix)",
                "Development shells (use devshell)",
                "Package management (use Nix/Pixi)"
            ]
        },
        "mise": {
            "consider_when": [
                "Non-Nix users need quick onboarding",
                "Runtime version switching during development",
                "CI environments without Nix"
            ],
            "do_not_use_for": [
                "ROS2 packages (no RoboStack equivalent)",
                "Reproducible builds (less rigorous than Nix)",
                "Primary development workflow"
            ]
        }
    },
    "adr_reference": "docs/adr/adr-003-version-management.md"
}
