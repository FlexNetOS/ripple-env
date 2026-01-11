{
  description = "ROS2 Humble development environment with Nix flakes and pixi";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    systems.url = "github:nix-systems/default";

    # Home-manager for user configuration
    home-manager = {
      url = "github:nix-community/home-manager";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    # Holochain overlay for P2P coordination (BUILDKIT_STARTER_SPEC.md L11)
    # Provides: holochain, hc, lair-keystore
    # NOTE: This overlay is loaded via fetchFromGitHub (not as flake input) because:
    #   1. The upstream repo is not a proper flake
    #   2. fetchFromGitHub allows pinning to a specific commit
    #   3. Avoids flake.lock sync issues
    # See: https://github.com/spartan-holochain-counsel/nix-overlay

    # P2-017: Agentic DevOps automation layer (BUILDKIT_STARTER_SPEC.md L301)
    # Provides: Autonomous DevOps and remediation capabilities
    # See: https://github.com/agenticsorg/devops
    agenticsorg-devops = {
      url = "github:agenticsorg/devops";
      flake = false;  # Not a flake, just source reference
    };

    # NixOS-WSL for WSL2 image generation
    # Provides: NixOS modules for Windows Subsystem for Linux
    # See: https://github.com/nix-community/NixOS-WSL
    nixos-wsl = {
      url = "github:nix-community/NixOS-WSL";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    inputs@{
      self,
      nixpkgs,
      flake-parts,
      systems,
      home-manager,
      nixos-wsl,
      ...
    }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import systems;

      # Flake-level outputs (not per-system)
      flake = {
        # Export library functions
        lib =
          (import ./lib {
            inherit (nixpkgs) lib;
            inherit inputs;
          })
          // {
            # Home-manager modules are not a standard flake output; expose them under lib
            # to avoid warnings like: "unknown flake output 'homeManagerModules'".
            homeManagerModules = {
              common = ./modules/common;
              linux = ./modules/linux;
              macos = ./modules/macos;

              # Combined module that auto-selects based on platform
              default =
                {
                  config,
                  lib,
                  pkgs,
                  ...
                }:
                {
                  imports = [
                    ./modules/common
                  ]
                  ++ lib.optionals pkgs.stdenv.isLinux [
                    ./modules/linux
                  ]
                  ++ lib.optionals pkgs.stdenv.isDarwin [
                    ./modules/macos
                  ];
                };
            };
          };

        # Export NixOS/Darwin modules (for system-level configuration)
        nixosModules.default = ./modules/linux;
        darwinModules.default = ./modules/macos;

        # NixOS configurations for image generation
        # Build commands:
        #   nix build .#nixosConfigurations.wsl-ros2.config.system.build.tarballBuilder
        #   nix build .#nixosConfigurations.iso-ros2.config.system.build.isoImage
        #   nix build .#nixosConfigurations.vm-ros2.config.system.build.vm
        nixosConfigurations = {
          # WSL2 image (Windows Subsystem for Linux)
          wsl-ros2 = import ./nix/images/wsl.nix {
            inherit inputs;
            pkgs = nixpkgs.legacyPackages.x86_64-linux;
            lib = nixpkgs.lib;
          };

          # ISO installer image
          iso-ros2 = import ./nix/images/iso.nix {
            inherit inputs;
            pkgs = nixpkgs.legacyPackages.x86_64-linux;
            lib = nixpkgs.lib;
          };

          # QEMU VM image
          vm-ros2 = import ./nix/images/vm.nix {
            inherit inputs;
            pkgs = nixpkgs.legacyPackages.x86_64-linux;
            lib = nixpkgs.lib;
          };
        };
      };

      # Per-system outputs
      perSystem =
        { pkgs, system, ... }:
        let
          # Configure nixpkgs with allowUnfree for packages like vault (BSL license)
          # Define the holochain source
          # Note: Using commit hash as the upstream repo has no tagged releases yet
          holochainSrc = inputs.nixpkgs.legacyPackages.${system}.fetchFromGitHub {
            owner = "spartan-holochain-counsel";
            repo = "nix-overlay";
            rev = "2a321bc7d6d94f169c6071699d9a89acd55039bb";  # Latest commit as of 2026-01-09
            sha256 = "sha256-LZkgXdLY+C+1CxynKzsdtM0g4gC0NJjPP3d24pHPyIU=";
          };

          # P2-017: AgenticsOrg DevOps automation layer reference
          # Source is available at: inputs.agenticsorg-devops
          # Usage: Provides autonomous DevOps and remediation capabilities
          # Documentation: ${inputs.agenticsorg-devops}/README.md
          agenticsorgDevopsSrc = inputs.agenticsorg-devops;

          # Apply Holochain overlay for P2P coordination
          pkgs = import inputs.nixpkgs {
            inherit system;
            config.allowUnfree = true;
            overlays = [
              # Holochain overlay - import the overlay function from the repository
              (import "${holochainSrc}/holochain-overlay/default.nix")
            ];
          };

          # Holochain packages from overlay (P0 - MANDATORY per BUILDKIT_STARTER_SPEC.md)
          # P3-006: Holochain reference tools (Phase 3 - Development tooling)
          #
          # The 'hc' CLI provides comprehensive development commands:
          #   - hc sandbox    : Generate and run test networks for development
          #   - hc scaffold   : Generate DNA, zome, and entry type templates
          #   - hc dna        : DNA operations (init, pack, unpack)
          #   - hc app        : hApp bundle operations (pack, unpack)
          #   - hc web-app    : Web hApp operations
          #
          # For additional launch capabilities:
          #   - Use 'hc sandbox' for local development environments
          #   - For production: holochain conductor with conductor.yaml config
          #   - Alternative: Install @holochain/hc-spin via npm for enhanced DX
          #
          # References:
          #   - Holochain Developer Docs: https://developer.holochain.org
          #   - hc CLI source: https://github.com/holochain/holochain (crates/hc)
          #   - Nix overlay: https://github.com/spartan-holochain-counsel/nix-overlay
          holochainPackages = with pkgs; [
            holochain       # Holochain conductor (agent-centric P2P runtime)
            hc              # Holochain dev CLI (scaffold/sandbox/pack/launch)
            lair-keystore   # Secure keystore for Holochain agent keys (cryptographic identity)
          ];

          inherit (pkgs.lib) optionalString optionals optionalAttrs;
          isDarwin = pkgs.stdenv.isDarwin;
          isLinux = pkgs.stdenv.isLinux;

          # Colcon defaults configuration
          colconDefaults = pkgs.writeText "defaults.yaml" ''
            build:
              cmake-args:
                - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
                - -DPython_FIND_VIRTUALENV=ONLY
                - -DPython3_FIND_VIRTUALENV=ONLY
                - -Wno-dev
                ${optionalString isDarwin "- -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"}
          '';

          # =================================================================
          # MODULAR STRUCTURE - Primary source for packages and commands
          # All definitions are in nix/ directory for maintainability
          # =================================================================

          # Import modular packages (nix/packages/)
          modularPackages = import ./nix/packages { inherit pkgs lib; };

          # Import modular commands (nix/commands/)
          modularCommands = import ./nix/commands { inherit pkgs; };

          # Import modular shell configuration (nix/shells/)
          modularShells = import ./nix/shells {
            inherit pkgs lib system colconDefaults;
            packages = modularPackages;
            commands = modularCommands;
          };

          # CUDA 13.x package set (latest available in nixpkgs)
          # Falls back to default cudaPackages if 13.1 unavailable
          cudaPkgs = pkgs.cudaPackages_13_1 or pkgs.cudaPackages_13 or pkgs.cudaPackages;

        in
        {
          # Development shell (main entry point)
          # Use standard `devShells` (and avoid the devshell flake module) so `nix flake check`
          # stays warning-free on newer Nix.
          devShells.default = pkgs.mkShell {
            # Include Holochain in default shell - P2P coordination is mandatory
            # Using modular imports from nix/packages/ and nix/commands/
            packages = modularPackages.defaultShell ++ modularCommands.defaultShell;
            COLCON_DEFAULTS_FILE = toString colconDefaults;
            EDITOR = "hx";
            VISUAL = "hx";

            # P2-017: AgenticsOrg DevOps source path
            AGENTICSORG_DEVOPS_SRC = toString agenticsorgDevopsSrc;

            shellHook = ''
              # Ensure TMPDIR is valid (fix for Codespaces/devcontainers)
              export TMPDIR=''${TMPDIR:-/tmp}
              [ -d "$TMPDIR" ] || export TMPDIR=/tmp
              mkdir -p "$TMPDIR" 2>/dev/null || true

              # Define stub functions for RoboStack activation scripts
              # These are called by conda post-link scripts but don't exist in plain bash
              noa_add_path() { :; }
              export -f noa_add_path 2>/dev/null || true

              # Initialize pixi environment
              if [ -f pixi.toml ]; then
                ${optionalString isDarwin ''
                  export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                ''}
                eval "$(pixi shell-hook 2>/dev/null)" || true
              fi

              # Keep startup fast for non-interactive shells (CI, `nix develop --command ...`).
              if [[ $- == *i* ]]; then
                echo ""
                echo "ROS2 Humble Development Environment"
                echo "=================================="
                echo "  Platform: ${if isDarwin then "macOS" else "Linux"} (${system})"
                echo ""
              fi
            '';
          };

          # Full-featured shell (slower initial download, more tools)
          devShells.full = pkgs.mkShell {
            # Using modular imports from nix/packages/ and nix/commands/
            packages = modularPackages.fullShell ++ modularCommands.fullShell;
            COLCON_DEFAULTS_FILE = toString colconDefaults;
            EDITOR = "hx";
            VISUAL = "hx";

            # Layer 3 Isolation Configuration (ARIA P0-001, P0-002)
            # DEFAULT_ISOLATION: Primary isolation runtime for containers/workloads
            # Options: "firecracker" (microVM), "kata" (lightweight VM), "sandbox-runtime" (process-level)
            DEFAULT_ISOLATION = "firecracker";

            # TOOL_ISOLATION: Isolation for MCP/AI tool execution
            # Options: "sandbox-runtime" (recommended), "kata", "none"
            TOOL_ISOLATION = "sandbox-runtime";

            shellHook = ''
              # Ensure TMPDIR is valid (fix for Codespaces/devcontainers)
              export TMPDIR=''${TMPDIR:-/tmp}
              [ -d "$TMPDIR" ] || export TMPDIR=/tmp
              mkdir -p "$TMPDIR" 2>/dev/null || true

              # Define stub functions for RoboStack activation scripts
              noa_add_path() { :; }
              export -f noa_add_path 2>/dev/null || true

              if [ -f pixi.toml ]; then
                ${optionalString isDarwin ''
                  export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                ''}
                eval "$(pixi shell-hook 2>/dev/null)" || true
              fi
              # Initialize direnv
              eval "$(direnv hook bash)"

              # Initialize zoxide
              eval "$(zoxide init bash)"

              # Initialize starship prompt
              eval "$(starship init bash)"

              # ROS2 environment info
              echo ""
              echo "ROS2 Humble Development Environment (Full)"
              echo "==========================================="
              echo "🤖 ROS2 Humble Development Environment"
              echo "======================================"
              echo "  Platform: ${if isDarwin then "macOS" else "Linux"} (${system})"
              echo "  Python (Nix): ${pkgs.python313.version} (for scripts/tools)"
              echo "  Python (ROS2): 3.11.x via Pixi/RoboStack"
              echo ""
              echo "Layer 3 Isolation (ARIA Audit):"
              echo "  DEFAULT_ISOLATION: $DEFAULT_ISOLATION"
              echo "  TOOL_ISOLATION:    $TOOL_ISOLATION"
              echo "  firecracker        - MicroVM isolation (ready)"
              echo "  kata               - Kata Containers (check: kata status)"
              echo "  sandbox-runtime    - Process sandbox (check: sandbox-runtime --version)"
              echo ""
              echo "Quick commands:"
              echo "  cb          - colcon build --symlink-install"
              echo "  ct          - colcon test"
              echo "  ros2-clean  - Clean build artifacts (--all for logs too)"
              echo "  ros2-ws     - Show workspace info"
              echo "  ros2-topics - List ROS2 topics"
              echo "  ros2-nodes  - List ROS2 nodes"
              echo ""
              echo "Development:"
              echo "  dev-check   - Run all checks (--fix to auto-fix)"
              echo "  fmt-nix     - Format all Nix files"
              echo "  pre-commit  - Git pre-commit checks"
              echo ""
              echo "AI assistants:"
              echo "  ai          - AI chat (aichat, lightweight)"
              echo "  pair        - AI pair programming (aider, git-integrated)"
              echo "  promptfoo   - LLM testing & evaluation"
              echo ""
              echo "AI infrastructure:"
              echo "  localai     - LocalAI server (start|stop|status|models)"
              echo "  agixt       - AGiXT platform (up|down|logs|status)"
              echo "  aios        - AIOS Kernel (install|start|stop|status)"
              echo ""
              echo "Infrastructure:"
              echo "  ipfs-ctl    - IPFS node (init|start|stop|status)"
              echo "  nats-ctl    - NATS server (start|stop|pub|sub)"
              echo "  prom-ctl    - Prometheus (start|stop|config)"
              echo "  vault-dev   - HashiCorp Vault dev mode"
              echo ""
              echo "Security:"
              echo "  sbom        - Generate SBOM (syft)"
              echo "  vuln-scan   - Vulnerability scan (grype/trivy)"
              echo "  sign-artifact - Sign with cosign"
              echo "  pki-cert    - PKI certificates (step-cli)"
              echo ""
              echo "Holochain (P2P) - P3-006 Reference Tools:"
              echo "  holochain       - Holochain conductor runtime"
              echo "  hc              - Holochain dev CLI"
              echo "    hc sandbox    - Generate and run test networks"
              echo "    hc scaffold   - Generate DNA/zome templates"
              echo "    hc dna        - DNA operations (init/pack/unpack)"
              echo "    hc app        - hApp bundle operations"
              echo "  lair-keystore   - Secure cryptographic keystore"
              echo ""
            '';
          };

          # CUDA-enabled shell for GPU workloads
          # Usage: nix develop .#cuda
          # Requires: NVIDIA GPU with drivers installed
          # Binary cache: https://cache.nixos-cuda.org
          devShells.cuda = pkgs.mkShell {
            # Using modular imports plus CUDA-specific packages
            packages = modularPackages.fullShell ++ modularCommands.fullShell ++ (with pkgs; [
              # CUDA Toolkit 13.x (or latest available)
              # See docs/CONFLICTS.md for version details
              cudaPkgs.cudatoolkit
              cudaPkgs.cudnn
              cudaPkgs.cutensor
              cudaPkgs.nccl
              cudaPkgs.cuda_cudart

              # GCC 13 pinned for CUDA compatibility
              # CUDA requires specific GCC versions for nvcc
              gcc13

              # GPU monitoring
              nvtopPackages.full
            ]);

            COLCON_DEFAULTS_FILE = toString colconDefaults;
            EDITOR = "hx";
            VISUAL = "hx";

            # CUDA environment variables
            CUDA_PATH = "${cudaPkgs.cudatoolkit}";
            # Pin compiler for CUDA compatibility
            CC = "${pkgs.gcc13}/bin/gcc";
            CXX = "${pkgs.gcc13}/bin/g++";

            shellHook = ''
              # Ensure TMPDIR is valid (fix for Codespaces/devcontainers)
              export TMPDIR=''${TMPDIR:-/tmp}
              [ -d "$TMPDIR" ] || export TMPDIR=/tmp
              mkdir -p "$TMPDIR" 2>/dev/null || true

              # Define stub functions for RoboStack activation scripts
              noa_add_path() { :; }
              export -f noa_add_path 2>/dev/null || true

              # Set up LD_LIBRARY_PATH for CUDA libraries
              export LD_LIBRARY_PATH="${cudaPkgs.cudatoolkit}/lib:${cudaPkgs.cudnn}/lib:$LD_LIBRARY_PATH"

              # Initialize pixi environment with CUDA feature
              if [ -f pixi.toml ]; then
                eval "$(pixi shell-hook -e cuda 2>/dev/null || pixi shell-hook 2>/dev/null)" || true
              fi

              # Initialize direnv
              eval "$(direnv hook bash)"

              # Initialize zoxide
              eval "$(zoxide init bash)"

              # Initialize starship prompt
              eval "$(starship init bash)"

              # Verify CUDA availability
              if command -v nvidia-smi &> /dev/null; then
                echo ""
                echo "ROS2 Humble + CUDA Development Environment"
                echo "==========================================="
                echo "  Platform: Linux (${system}) with NVIDIA GPU"
                nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null | head -1 | while read line; do
                  echo "  GPU: $line"
                done
                echo "  CUDA: ${cudaPkgs.cudatoolkit.version}"
                echo "  GCC: $(${pkgs.gcc13}/bin/gcc --version | head -1)"
                echo "  Python (Nix): ${pkgs.python313.version}"
                echo "  Python (ROS2): via Pixi 3.11.x"
                echo ""
                echo "PyTorch CUDA verification:"
                echo "  python -c \"import torch; print(torch.cuda.is_available())\""
                echo ""
                echo "AI assistants:"
                echo "  ai        - AI chat (aichat, lightweight)"
                echo "  pair      - AI pair programming (aider, git-integrated)"
                echo "  promptfoo - LLM testing & evaluation (robot command parsing)"
                echo ""
              else
                echo ""
                echo "⚠️  Warning: nvidia-smi not found"
                echo "   CUDA toolkit is available but GPU drivers may not be installed."
                echo "   Install NVIDIA drivers on your host system."
                echo ""
              fi
            '';
          };

          # NOTE: Legacy devshells.default was removed - it required the devshell
          # flake-parts module which isn't imported. Use devShells.default instead.
          # The command aliases (cb, ct, ctr, etc.) are available as executable commands in devShells.default.

          # Identity & Auth shell for Keycloak/Vaultwarden development (Linux only)
          # Usage: nix develop .#identity
          # Heavy dependencies: Java 21, PostgreSQL
          devShells.identity = pkgs.mkShell {
            # Using modular imports plus identity-specific packages
            packages = modularPackages.base ++ modularPackages.linux ++ modularCommands.core ++ (with pkgs; [
              # Identity & Access Management
              keycloak             # OAuth2/OIDC identity provider (Java 21)
              vaultwarden          # Bitwarden-compatible password manager (Rust)

              # Database backends
              postgresql_15        # PostgreSQL for Keycloak/Vaultwarden
              sqlite               # SQLite for lightweight Vaultwarden

              # Java runtime (required by Keycloak)
              jdk21_headless       # Java 21 LTS (headless for servers)

              # Database tools
              pgcli                # PostgreSQL CLI with autocomplete
            ]);

            COLCON_DEFAULTS_FILE = toString colconDefaults;
            EDITOR = "hx";
            VISUAL = "hx";

            # Java environment
            JAVA_HOME = "${pkgs.jdk21_headless}";

            shellHook = ''
              # Ensure TMPDIR is valid
              export TMPDIR=''${TMPDIR:-/tmp}
              [ -d "$TMPDIR" ] || export TMPDIR=/tmp

              # Define stub functions for RoboStack activation scripts
              noa_add_path() { :; }
              export -f noa_add_path 2>/dev/null || true

              if [ -f pixi.toml ]; then
                ${optionalString isDarwin ''
                  export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                ''}
                eval "$(pixi shell-hook 2>/dev/null)" || true
              fi

              echo ""
              echo "🔐 Identity & Auth Development Environment"
              echo "=========================================="
              echo "  Platform: Linux (${system})"
              echo "  Java: $(java -version 2>&1 | head -1)"
              echo "  PostgreSQL: ${pkgs.postgresql_15.version}"
              echo ""
              echo "Available services:"
              echo "  keycloak        - OAuth2/OIDC identity provider"
              echo "  vaultwarden     - Bitwarden-compatible password manager"
              echo ""
              echo "Quick start:"
              echo "  # Start PostgreSQL (for Keycloak)"
              echo "  initdb -D ./pgdata && pg_ctl -D ./pgdata start"
              echo ""
              echo "  # Start Keycloak in dev mode"
              echo "  keycloak start-dev --http-port=8080"
              echo ""
              echo "  # Start Vaultwarden (SQLite)"
              echo "  vaultwarden"
              echo ""
            '';
          };
        };
    };
}
