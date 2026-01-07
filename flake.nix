{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    systems.url = "github:nix-systems/default";
    devshell.url = "github:numtide/devshell";
  };

  outputs =
    inputs@{
      flake-parts,
      systems,
      devshell,
      ...
    }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import systems;
      imports = [
        devshell.flakeModule
      ];
      perSystem =
        { pkgs, ... }:
        let
          inherit (pkgs.lib)
            optionalString
            ;

          isDarwin = pkgs.stdenv.isDarwin;

          colconDefaults = pkgs.writeText "defaults.yaml" ''
            build:
              cmake-args:
                - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
                - -DPython_FIND_VIRTUALENV=ONLY
                - -DPython3_FIND_VIRTUALENV=ONLY
                - -Wno-dev
                ${optionalString isDarwin "- -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"}
          '';

          # LazyVim setup script
          lazyvimSetup = pkgs.writeShellScript "setup-lazyvim.sh" ''
            NVIM_CONFIG="$HOME/.config/nvim"
            
            # Validate the config path before proceeding
            if [ -z "$NVIM_CONFIG" ] || [ -z "$HOME" ]; then
              echo "ERROR: Invalid configuration path" >&2
              exit 1
            fi
            
            # Ensure the path is within the user's home directory
            RESOLVED_PATH=$(cd "$(dirname "$NVIM_CONFIG")" 2>/dev/null && pwd -P)/$(basename "$NVIM_CONFIG") || true
            if [ -n "$RESOLVED_PATH" ]; then
              case "$RESOLVED_PATH" in
                "$HOME"*)
                  # Path is within home directory, safe to proceed
                  ;;
                *)
                  echo "ERROR: Config path must be within home directory" >&2
                  exit 1
                  ;;
              esac
            fi
            
            if [ ! -d "$NVIM_CONFIG" ]; then
              echo "Setting up LazyVim..."
              git clone https://github.com/LazyVim/starter "$NVIM_CONFIG"
              # Remove .git directory from the cloned starter (safe since we just created it)
              if [ -d "$NVIM_CONFIG/.git" ]; then
                rm -rf "$NVIM_CONFIG/.git"
              fi
              echo "LazyVim starter cloned to $NVIM_CONFIG"
              echo "Run 'nvim' to complete the setup."
            else
              echo "Neovim config already exists at $NVIM_CONFIG"
            fi
          '';
        in
        {
          devshells.default = {
            env = [
              {
                name = "COLCON_DEFAULTS_FILE";
                value = toString colconDefaults;
              }
            ];
            devshell = {
              packages = with pkgs; [
                pixi
                neovim
                git
                gcc
                gnumake
                ripgrep
                fd
                nodejs
                tree-sitter
              ];
              commands = [
                {
                  name = "setup-lazyvim";
                  help = "Initialize LazyVim configuration";
                  command = toString lazyvimSetup;
                }
              ];
              startup.activate.text = ''
                if [ -f pixi.toml ]; then
                  ${optionalString isDarwin ''
                    export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                  ''}
                  eval "$(pixi shell-hook)";
                fi
              '';
              motd = "";
            };
          };
        };
    };
}
