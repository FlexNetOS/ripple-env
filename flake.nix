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
              ];
              startup.activate.text = ''
                # ROS2 Humble Environment Activation Script
                # This script initializes the pixi-managed ROS2 development environment

                _ros2_activate() {
                  # Check if pixi.toml exists in the current directory
                  if [ ! -f pixi.toml ]; then
                    echo "[ros2-env] INFO: pixi.toml not found in $(pwd), skipping pixi activation" >&2
                    return 0
                  fi

                  # Verify pixi command is available
                  if ! command -v pixi >/dev/null 2>&1; then
                    echo "[ros2-env] ERROR: pixi command not found in PATH" >&2
                    echo "[ros2-env] Please ensure pixi is installed and available" >&2
                    return 1
                  fi

                  # Check if pixi environment has been initialized
                  if [ ! -d ".pixi/envs/default" ]; then
                    echo "[ros2-env] WARNING: Pixi environment not initialized" >&2
                    echo "[ros2-env] Run 'pixi install' to set up the environment" >&2
                    return 1
                  fi

                  ${optionalString isDarwin ''
                    # macOS: Configure dynamic library fallback path for ROS2
                    local pixi_lib_path="$PWD/.pixi/envs/default/lib"
                    if [ -d "$pixi_lib_path" ]; then
                      export DYLD_FALLBACK_LIBRARY_PATH="$pixi_lib_path:''${DYLD_FALLBACK_LIBRARY_PATH:-}"
                    else
                      echo "[ros2-env] WARNING: Pixi library path not found: $pixi_lib_path" >&2
                      echo "[ros2-env] Some ROS2 libraries may not load correctly" >&2
                    fi
                  ''}

                  # Generate and execute pixi shell-hook
                  local pixi_hook
                  if ! pixi_hook=$(pixi shell-hook 2>&1); then
                    echo "[ros2-env] ERROR: Failed to generate pixi shell-hook" >&2
                    echo "[ros2-env] pixi output: $pixi_hook" >&2
                    return 1
                  fi

                  # Validate shell-hook output is not empty
                  if [ -z "$pixi_hook" ]; then
                    echo "[ros2-env] ERROR: pixi shell-hook returned empty output" >&2
                    echo "[ros2-env] This may indicate a pixi configuration issue" >&2
                    return 1
                  fi

                  # Execute the shell-hook to activate the environment
                  if ! eval "$pixi_hook"; then
                    echo "[ros2-env] ERROR: Failed to evaluate pixi shell-hook" >&2
                    return 1
                  fi

                  echo "[ros2-env] ROS2 Humble environment activated successfully" >&2
                  return 0
                }

                # Run activation and capture result
                _ros2_activate
                _ros2_activate_result=$?

                # Clean up the function to avoid polluting the shell namespace
                unset -f _ros2_activate

                # Return the activation result (non-fatal to allow shell to continue)
                if [ $_ros2_activate_result -ne 0 ]; then
                  echo "[ros2-env] Environment activation failed (exit code: $_ros2_activate_result)" >&2
                  echo "[ros2-env] The shell will continue, but ROS2 tools may not work correctly" >&2
                fi
                unset _ros2_activate_result
              '';
              motd = "";
            };
          };
        };
    };
}
