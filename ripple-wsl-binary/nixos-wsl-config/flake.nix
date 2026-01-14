{
  description = "FlexNetOS - NixOS WSL Distribution";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    nixos-wsl.url = "github:nix-community/NixOS-WSL";
    flake-utils.url = "github:numtide/flake-utils";
    
    # FlexNetOS specific inputs
    holonix.url = "github:holochain/holonix";
    
    # Additional overlays
    nur.url = "github:nix-community/NUR";
  };

  outputs = { self, nixpkgs, nixos-wsl, flake-utils, holonix, nur, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
          overlays = [
            nixos-wsl.overlays.default
          ];
        };

        # WSL-specific configuration
        wslConfig = {
          wsl = {
            enable = true;
            defaultUser = "flexnet";
            automountPath = "/mnt";
            docker-desktop.enable = false;
          };
        };

      in {
        nixosConfigurations.flexnetos-wsl = nixpkgs.lib.nixosSystem {
          inherit system;
          modules = [
            nixos-wsl.nixosModules.wsl
            ./configuration.nix
            ./modules/flexnetos-core.nix
            ./modules/flexnetos-services.nix
            ./modules/flexnetos-security.nix
            ./modules/flexnetos-monitoring.nix
          ];
          specialArgs = { inherit wslConfig; };
        };

        packages.default = self.nixosConfigurations.flexnetos-wsl.config.system.build.toplevel;
        
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            nixos-generators
            wslu
            git
            curl
            jq
          ];
        };
      });
}
