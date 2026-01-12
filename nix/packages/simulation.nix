# Simulation packages for robotics testing
# Provides tools for visualization, 3D rendering, and physics simulation
#
# Usage:
#   packages.simulation - Simulation stack
#
# Parameters:
#   rosDistro - ROS2 distribution (humble, iron, rolling). Default: humble
{ pkgs, lib ? pkgs.lib, rosDistro ? "humble", ... }:

with pkgs; [
  # 3D visualization and rendering
  ogre               # 3D graphics engine (used by RViz)
  assimp             # 3D model import library

  # OpenGL/Vulkan support
  mesa               # OpenGL implementation
  glxinfo            # OpenGL info utility

  # X11/Wayland display
  xorg.xeyes         # Simple X11 test
  xdotool            # X11 automation tool

  # Physics simulation support
  bullet             # Physics library
  ode                # Open Dynamics Engine

  # Robot visualization helpers
  graphviz           # Graph visualization (TF tree rendering)
  ffmpeg             # Video capture/encoding
  imagemagick        # Image manipulation

  # Testing utilities
  xvfb-run           # Virtual framebuffer for headless testing
]

# Note: Full simulation environments should be installed via robostack/pixi.
# The rosDistro parameter (default: "humble") determines the package prefix:
#   - ros-${rosDistro}-rviz2
#   - ros-${rosDistro}-gazebo-ros-pkgs
#   - ros-${rosDistro}-ros-gz-bridge
#   - ros-${rosDistro}-ros-gz-sim
#   - ros-${rosDistro}-nav2-bringup
#   - ros-${rosDistro}-slam-toolbox
#   - ros-${rosDistro}-turtlebot3-gazebo
# Install via:
#   pixi add ros-${rosDistro}-<package-name>
# Example for Humble: pixi add ros-humble-rviz2
# Example for Iron:   pixi add ros-iron-rviz2
#
# For Gazebo simulation, see:
#   https://gazebosim.org/docs/harmonic/install_ubuntu
#   https://navigation.ros.org/getting_started/index.html
