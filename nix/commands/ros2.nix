# ROS2-specific command wrappers
# Includes: ros2-clean, ros2-ws, ros2-topics, ros2-nodes
# Extracted from flake.nix for modular organization
{ pkgs, ... }:

[
            # Clean ROS2 build artifacts
            (pkgs.writeShellScriptBin "ros2-clean" ''
              # Clean ROS2/colcon build artifacts
              # Usage: ros2-clean [--all]
              set -e

              if [ "$1" = "--all" ] || [ "$1" = "-a" ]; then
                echo "[ros2-clean] Removing all build artifacts..."
                rm -rf build/ install/ log/ .colcon_build_status
                echo "[ros2-clean] Cleaned: build/, install/, log/"
              else
                echo "[ros2-clean] Removing build and install directories..."
                rm -rf build/ install/
                echo "[ros2-clean] Cleaned: build/, install/"
                echo "[ros2-clean] Use --all to also remove log/"
              fi
            '')
            # Show ROS2 workspace information
            (pkgs.writeShellScriptBin "ros2-ws" ''
              # Display ROS2 workspace information
              echo "ROS2 Workspace Information"
              echo "=========================="
              echo ""

              # ROS2 environment
              echo "Environment:"
              if [ -n "$ROS_DISTRO" ]; then
                echo "  ROS_DISTRO: $ROS_DISTRO"
              else
                echo "  ROS_DISTRO: (not set)"
              fi
              if [ -n "$RMW_IMPLEMENTATION" ]; then
                echo "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
              fi
              if [ -n "$ROS_DOMAIN_ID" ]; then
                echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
              fi
              echo ""

              # Packages
              echo "Packages:"
              if [ -d "src" ]; then
                pkg_count=$(find src -name "package.xml" 2>/dev/null | wc -l)
                echo "  Source packages: $pkg_count"
                if [ "$pkg_count" -gt 0 ] && [ "$pkg_count" -le 20 ]; then
                  find src -name "package.xml" -exec dirname {} \; | sed 's|.*/||' | sort | sed 's/^/    - /'
                fi
              else
                echo "  No src/ directory found"
              fi
              echo ""

              # Build status
              echo "Build Status:"
              if [ -d "build" ]; then
                built_count=$(ls -1 build/ 2>/dev/null | wc -l)
                echo "  Built packages: $built_count"
              else
                echo "  Not built yet (run 'cb' to build)"
              fi
              if [ -d "install" ]; then
                echo "  Install directory: exists"
              fi
              echo ""

              # Pixi environment
              echo "Pixi Environment:"
              if [ -f "pixi.toml" ]; then
                echo "  pixi.toml: found"
                if [ -d ".pixi/envs/default" ]; then
                  echo "  Environment: initialized"
                else
                  echo "  Environment: not initialized (run 'pixi install')"
                fi
              else
                echo "  pixi.toml: not found"
              fi
            '')
            # List ROS2 topics with filtering
            (pkgs.writeShellScriptBin "ros2-topics" ''
              # List ROS2 topics with optional filtering
              # Usage: ros2-topics [filter]
              if ! command -v ros2 >/dev/null 2>&1; then
                echo "Error: ros2 command not found" >&2
                echo "Make sure ROS2 environment is sourced" >&2
                exit 1
              fi

              if [ -n "$1" ]; then
                echo "ROS2 Topics matching '$1':"
                ros2 topic list | grep -i "$1" || echo "  (no matches)"
              else
                echo "ROS2 Topics:"
                ros2 topic list
              fi
            '')
            # Quick ROS2 node inspection
            (pkgs.writeShellScriptBin "ros2-nodes" ''
              # List ROS2 nodes with optional filtering
              # Usage: ros2-nodes [filter]
              if ! command -v ros2 >/dev/null 2>&1; then
                echo "Error: ros2 command not found" >&2
                echo "Make sure ROS2 environment is sourced" >&2
                exit 1
              fi

              if [ -n "$1" ]; then
                echo "ROS2 Nodes matching '$1':"
                ros2 node list | grep -i "$1" || echo "  (no matches)"
              else
                echo "ROS2 Nodes:"
                ros2 node list
              fi
            '')
          ];
