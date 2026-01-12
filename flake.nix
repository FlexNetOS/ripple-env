{
  description = "ROS2 Humble development environment with Nix flakes and pixi";

  # =================================================================
  # NIX EVALUATION OPTIMIZATION
  # =================================================================
  # Performance tips for fast evaluation:
  # 1. Use `flake = false` for inputs that don't need flake features
  # 2. Pin inputs to specific commits/tags when possible
  # 3. Use `follows` to deduplicate nixpkgs across inputs
  # 4. Lazy-load heavy features (CUDA, identity shells) only on Linux
  #
  # Benchmark: nix flake show --no-build 2>&1 | tail -1
  # Target: < 10 seconds for flake evaluation
  # =================================================================

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";

    # Systems input - minimal overhead
    systems.url = "github:nix-systems/default";

    # Home-manager for user configuration
    # Uses `follows` to avoid duplicate nixpkgs evaluation
    home-manager = {
      url = "github:nix-community/home-manager";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    # Holochain overlay for P2P coordination (BUILDKIT_STARTER_SPEC.md L11)
    # NOTE: Loaded via fetchFromGitHub to pin specific commit (see perSystem)
    # This avoids adding another flake input with its own nixpkgs
    # See: https://github.com/spartan-holochain-counsel/nix-overlay

    # P2-017: Agentic DevOps automation layer (BUILDKIT_STARTER_SPEC.md L301)
    # flake = false: Only need the source, not flake outputs
    agenticsorg-devops = {
      url = "github:agenticsorg/devops";
      flake = false;
    };

    # NixOS-WSL for WSL2 image generation
    # Uses `follows` to avoid duplicate nixpkgs evaluation
    nixos-wsl = {
      url = "github:nix-community/NixOS-WSL";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  # =================================================================
  # NIX SETTINGS FOR PERFORMANCE
  # =================================================================
  nixConfig = {
    # Binary caches for faster builds
    extra-substituters = [
      "https://cache.nixos.org"
      "https://nix-community.cachix.org"
      "https://cuda-maintainers.cachix.org"
    ];
    extra-trusted-public-keys = [
      "cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY="
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
      "cuda-maintainers.cachix.org-1:0dq3bujKpuEPMCX6U4WylrUDZ9JyUG0VpVZa7CNfq5E="
    ];
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
            homeManagerModules = {
              common = ./modules/common;
              linux = ./modules/linux;
              macos = ./modules/macos;

              default =
                { config, lib, pkgs, ... }:
                {
                  imports = [ ./modules/common ]
                    ++ lib.optionals pkgs.stdenv.isLinux [ ./modules/linux ]
                    ++ lib.optionals pkgs.stdenv.isDarwin [ ./modules/macos ];
                };
            };
          };

        # Export NixOS/Darwin modules
        nixosModules.default = ./modules/linux;
        darwinModules.default = ./modules/macos;

        # NixOS configurations for image generation
        nixosConfigurations = {
          wsl-ros2 = import ./nix/images/wsl.nix {
            inherit inputs;
            pkgs = nixpkgs.legacyPackages.x86_64-linux;
            lib = nixpkgs.lib;
          };

          iso-ros2 = import ./nix/images/iso.nix {
            inherit inputs;
            pkgs = nixpkgs.legacyPackages.x86_64-linux;
            lib = nixpkgs.lib;
          };

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
          # Holochain overlay source (pinned commit)
          holochainSrc = inputs.nixpkgs.legacyPackages.${system}.fetchFromGitHub {
            owner = "spartan-holochain-counsel";
            repo = "nix-overlay";
            rev = "2a321bc7d6d94f169c6071699d9a89acd55039bb";
            sha256 = "sha256-LZkgXdLY+C+1CxynKzsdtM0g4gC0NJjPP3d24pHPyIU=";
          };

          # P2-017: AgenticsOrg DevOps source
          agenticsorgDevopsSrc = inputs.agenticsorg-devops;

          # Configure nixpkgs with Holochain overlay
          pkgs = import inputs.nixpkgs {
            inherit system;
            config.allowUnfree = true;
            overlays = [
              (import "${holochainSrc}/holochain-overlay/default.nix")
            ];
          };

          inherit (pkgs.lib) optionalString;
          isDarwin = pkgs.stdenv.isDarwin;

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
          # MODULAR STRUCTURE - All definitions in nix/ directory
          # Benefits: Faster evaluation, easier reviews, fewer conflicts
          # =================================================================

          # Import modular packages (nix/packages/)
          modularPackages = import ./nix/packages { inherit pkgs; lib = pkgs.lib; };

          # Import modular commands (nix/commands/)
          modularCommands = import ./nix/commands { inherit pkgs; };

          # Import modular shells (nix/shells/)
          # Contains: default, full, cuda, identity shells
          modularShells = import ./nix/shells {
            inherit pkgs system colconDefaults;
            lib = pkgs.lib;
            packages = modularPackages;
            commands = modularCommands;
          };

        in
        {
          # Development shells - use modular structure from nix/shells/
          # See nix/shells/default.nix for shell definitions
          devShells = modularShells // {
            # Override default to add AgenticsOrg DevOps source path
            default = modularShells.default.overrideAttrs (old: {
              AGENTICSORG_DEVOPS_SRC = toString agenticsorgDevopsSrc;
            });
          };
        };
    };
}
