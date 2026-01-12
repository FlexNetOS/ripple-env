# Shell definitions aggregator
# Usage in flake.nix:
#   shells = import ./nix/shells { inherit pkgs lib system colconDefaults; };
#   devShells = shells;
#
# This module defines all development shells for the project:
#   - minimal: Fastest possible startup (CI, scripting)
#   - default: Standard shell for fast startup
#   - full: All tools included
#   - cuda: GPU/CUDA development (Linux only)
#   - identity: Keycloak/Vaultwarden development (Linux only)
#
# Performance notes:
#   - Heavy shells (cuda, identity) are only defined on Linux
#   - Use 'nix develop .#minimal' for CI/scripting where speed is critical
#   - Use feature flags in nix/packages/dev-tools.nix for customization
{ pkgs, lib, system, colconDefaults, packages, commands, ... }:

let
  inherit (pkgs.stdenv) isDarwin isLinux;

  common = import ./common.nix {
    inherit pkgs lib colconDefaults isDarwin system;
  };

  # Base shell hook
  baseHook = common.baseShellHook;

  # Core shells (available on all platforms)
  coreShells = {
    # Minimal shell - fastest possible startup for CI/scripting
    # Target: < 1 second startup, < 5 second evaluation
    minimal = pkgs.mkShell {
      packages = packages.minimalShell;
      inherit (common.env) COLCON_DEFAULTS_FILE;

      shellHook = ''
        # Minimal hook - no extra initialization
        export TMPDIR=''${TMPDIR:-/tmp}
      '';
    };

    # Default shell - balanced for daily development
    default = pkgs.mkShell {
      packages = packages.defaultShell ++ commands.defaultShell;
      inherit (common.env) COLCON_DEFAULTS_FILE EDITOR VISUAL;

      # P2-017: AgenticsOrg DevOps source path (set by caller)
      # AGENTICSORG_DEVOPS_SRC = toString agenticsorgDevopsSrc;

      shellHook = baseHook + common.defaultShellHook;
    };

    # Full shell - all tools included
    full = pkgs.mkShell {
      packages = packages.fullShell ++ commands.fullShell;
      inherit (common.env) COLCON_DEFAULTS_FILE EDITOR VISUAL;

      # Layer 3 Isolation Configuration (ARIA P0-001, P0-002)
      DEFAULT_ISOLATION = "firecracker";
      TOOL_ISOLATION = "sandbox-runtime";

      shellHook = baseHook + common.fullShellHook pkgs.python313.version;
    };
  };

  # Linux-only shells (CUDA, identity) - only evaluated on Linux
  # This prevents unnecessary evaluation overhead on macOS
  linuxOnlyShells = lib.optionalAttrs isLinux (
    let
      # CUDA 13.x package set (latest available in nixpkgs)
      # Only evaluated when actually building a CUDA shell
      cudaPkgs = pkgs.cudaPackages_13_1 or pkgs.cudaPackages_13 or pkgs.cudaPackages;
    in {
      # CUDA-enabled shell for GPU workloads
      # Usage: nix develop .#cuda
      # Requires: NVIDIA GPU with drivers installed
      # Binary cache: https://cache.nixos-cuda.org
      cuda = pkgs.mkShell {
        packages = packages.fullShell ++ commands.fullShell ++ (with pkgs; [
          # CUDA Toolkit 13.x (or latest available)
          cudaPkgs.cudatoolkit
          cudaPkgs.cudnn
          cudaPkgs.cutensor
          cudaPkgs.nccl
          cudaPkgs.cuda_cudart

          # GCC 13 pinned for CUDA compatibility
          gcc13

          # GPU monitoring
          nvtopPackages.full
        ]);

        inherit (common.env) COLCON_DEFAULTS_FILE EDITOR VISUAL;

        # CUDA environment variables
        CUDA_PATH = "${cudaPkgs.cudatoolkit}";
        CUDNN_PATH = "${cudaPkgs.cudnn}";

        # Pin compiler for CUDA compatibility
        CC = "${pkgs.gcc13}/bin/gcc";
        CXX = "${pkgs.gcc13}/bin/g++";

        shellHook = baseHook + common.cudaShellHook
          cudaPkgs.cudatoolkit.version
          pkgs.gcc13
          pkgs.python313.version;
      };

      # Identity & Auth shell for Keycloak/Vaultwarden development
      # Usage: nix develop .#identity
      # Heavy dependencies: Java 21, PostgreSQL
      identity = pkgs.mkShell {
        packages = packages.base ++ packages.linux ++ commands.core ++ (with pkgs; [
          # Identity & Access Management
          keycloak
          vaultwarden

          # Database backends
          postgresql_15
          sqlite

          # Java runtime (required by Keycloak)
          jdk21_headless

          # Database tools
          pgcli
        ]);

        inherit (common.env) COLCON_DEFAULTS_FILE EDITOR VISUAL;

        # Java environment
        JAVA_HOME = "${pkgs.jdk21_headless}";

        shellHook = baseHook + common.identityShellHook
          "${pkgs.jdk21_headless}"
          pkgs.postgresql_15.version;
      };
    }
  );

in
# Merge core shells with platform-specific shells
coreShells // linuxOnlyShells
