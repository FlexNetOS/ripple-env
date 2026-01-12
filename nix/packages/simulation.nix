# Simulation packages for robotics testing
# Provides tools for visualization, 3D rendering, and physics simulation
#
# Usage:
#   packages.simulation - Simulation stack
{ pkgs, ... }:

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

# Note: Full simulation environments should be installed via robostack/pixi:
#   - ros-humble-rviz2
#   - ros-humble-gazebo-ros-pkgs
#   - ros-humble-ros-gz-bridge
#   - ros-humble-ros-gz-sim
#   - ros-humble-nav2-bringup
#   - ros-humble-slam-toolbox
#   - ros-humble-turtlebot3-gazebo
# Install via:
#   pixi add ros-humble-<package-name>
#
# For Gazebo simulation, see:
#   https://gazebosim.org/docs/harmonic/install_ubuntu
#   https://navigation.ros.org/getting_started/index.html
