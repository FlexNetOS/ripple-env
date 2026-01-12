{
  description = "ROS2 Humble development environment with Nix flakes and pixi";

  inputs = {
    # Supply Chain Security: Dual nixpkgs strategy
    # - nixpkgs-stable: For production builds and NixOS images (pinned release)
    # - nixpkgs: For development (unstable, more packages)
    # See: docs/SUPPLY_CHAIN_SECURITY.md
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    nixpkgs-stable.url = "github:nixos/nixpkgs/nixos-24.11";

    flake-parts.url = "github:hercules-ci/flake-parts";
    systems.url = "github:nix-systems/default";

    # Home-manager for user configuration
    home-manager = {
      url = "github:nix-community/home-manager";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    # Home-manager stable variant for production
    home-manager-stable = {
      url = "github:nix-community/home-manager/release-24.11";
      inputs.nixpkgs.follows = "nixpkgs-stable";
    };

    # Holochain overlay for P2P coordination (BUILDKIT_STARTER_SPEC.md L11)
    # NOTE: Loaded via fetchFromGitHub to pin specific commit
    # See: https://github.com/spartan-holochain-counsel/nix-overlay

    # P2-017: Agentic DevOps automation layer (BUILDKIT_STARTER_SPEC.md L301)
    agenticsorg-devops = {
      url = "github:agenticsorg/devops";
      flake = false;
    };

    # NixOS-WSL for WSL2 image generation
    nixos-wsl = {
      url = "github:nix-community/NixOS-WSL";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    # NixOS-WSL stable variant for production images
    nixos-wsl-stable = {
      url = "github:nix-community/NixOS-WSL";
      inputs.nixpkgs.follows = "nixpkgs-stable";
    };
  };

  outputs =
    inputs@{
      self,
      nixpkgs,
      nixpkgs-stable,
      flake-parts,
      systems,
      home-manager,
      home-manager-stable,
      nixos-wsl,
      nixos-wsl-stable,
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
        # Development images (nixos-unstable)
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

          # Production images (nixos-stable) - Supply Chain Security
          # Use these for production deployments with stable, well-tested packages
          wsl-ros2-stable = import ./nix/images/wsl.nix {
            inherit inputs;
            pkgs = nixpkgs-stable.legacyPackages.x86_64-linux;
            lib = nixpkgs-stable.lib;
            isStable = true;
          };

          iso-ros2-stable = import ./nix/images/iso.nix {
            inherit inputs;
            pkgs = nixpkgs-stable.legacyPackages.x86_64-linux;
            lib = nixpkgs-stable.lib;
            isStable = true;
          };

          vm-ros2-stable = import ./nix/images/vm.nix {
            inherit inputs;
            pkgs = nixpkgs-stable.legacyPackages.x86_64-linux;
            lib = nixpkgs-stable.lib;
            isStable = true;
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

          # Stable packages for production shells
          pkgsStable = import inputs.nixpkgs-stable {
            inherit system;
            config.allowUnfree = true;
          };

          # Stable shell packages (subset for production use)
          stableShellPackages = import ./nix/packages { pkgs = pkgsStable; lib = pkgsStable.lib; };
          stableShellCommands = import ./nix/commands { pkgs = pkgsStable; };

        in
        {
          # Development shells - use modular structure from nix/shells/
          # See nix/shells/default.nix for shell definitions
          devShells = modularShells // {
            # Override default to add AgenticsOrg DevOps source path
            default = modularShells.default.overrideAttrs (old: {
              AGENTICSORG_DEVOPS_SRC = toString agenticsorgDevopsSrc;
            });

            # Stable shell for production work (nixos-24.11)
            # Use: nix develop .#stable
            # Benefits: Well-tested packages, security updates, predictable behavior
            stable = pkgsStable.mkShell {
              packages = stableShellPackages.defaultShell ++ stableShellCommands.defaultShell;
              COLCON_DEFAULTS_FILE = colconDefaults;
              EDITOR = "nvim";
              VISUAL = "nvim";
              NIXPKGS_CHANNEL = "stable";

              shellHook = ''
                echo "╔══════════════════════════════════════════════════════════╗"
                echo "║  ROS2 Humble Environment (STABLE - nixos-24.11)          ║"
                echo "║  Supply chain verified with pinned dependencies          ║"
                echo "╚══════════════════════════════════════════════════════════╝"
                echo ""
                echo "Channel: nixos-24.11 (stable)"
                echo "Use 'nix develop .#default' for latest packages"
              '';
            };
          };
        };
    };
}
