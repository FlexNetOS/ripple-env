# Offline Mode Configuration Module
#
# This module provides configuration options for running ripple-env
# in offline or air-gapped environments.
#
# Usage:
#   1. Set RIPPLE_OFFLINE=1 environment variable
#   2. Or use: nix develop .#offline
#
{ lib, pkgs, ... }:

let
  # Check if offline mode is enabled via environment
  offlineMode = builtins.getEnv "RIPPLE_OFFLINE" == "1";

  # Default local cache paths
  defaultCachePaths = {
    nix = "$HOME/.cache/ripple-env/nix-store";
    pixi = "$HOME/.cache/ripple-env/pixi";
    docker = "$HOME/.cache/ripple-env/docker";
    models = "$HOME/.cache/ripple-env/models";
  };
in
{
  # Offline mode configuration options
  offline = {
    enable = offlineMode;

    # Local Nix binary cache configuration
    nix = {
      # Local substituter path (file:// or http://)
      localSubstituter = lib.mkDefault "file://${defaultCachePaths.nix}";

      # Disable remote substituters when offline
      disableRemoteSubstituters = lib.mkDefault offlineMode;
    };

    # Pixi/Conda mirror configuration
    pixi = {
      # Local conda channel mirror
      localChannel = lib.mkDefault "${defaultCachePaths.pixi}/channels";

      # Disable remote channels when offline
      disableRemoteChannels = lib.mkDefault offlineMode;
    };

    # Docker registry mirror configuration
    docker = {
      # Local registry mirror (e.g., localhost:5000)
      localRegistry = lib.mkDefault "localhost:5000";

      # Path to pre-pulled images
      imageCachePath = lib.mkDefault defaultCachePaths.docker;
    };

    # AI model cache configuration
    models = {
      # Local model storage path
      cachePath = lib.mkDefault defaultCachePaths.models;

      # HuggingFace offline mode
      huggingfaceOffline = lib.mkDefault offlineMode;
    };
  };

  # Environment variables for offline mode
  shellHook = lib.optionalString offlineMode ''
    echo "Offline mode enabled"

    # Nix offline settings
    export NIX_CONFIG="substituters = file://${defaultCachePaths.nix}
    fallback = false"

    # HuggingFace offline mode
    export HF_HUB_OFFLINE=1
    export TRANSFORMERS_OFFLINE=1

    # Pixi offline mode
    export PIXI_OFFLINE=1

    # Docker offline hints
    export DOCKER_BUILDKIT=0

    # Disable telemetry and updates
    export DO_NOT_TRACK=1
    export PIXI_NO_UPDATE_CHECK=1
  '';
}
