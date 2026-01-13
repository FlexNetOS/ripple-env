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
{ pkgs, ... }:

# NOTE: These packages require the holochain overlay to be applied to pkgs
# The overlay is applied in flake.nix before importing this file
with pkgs; [
  holochain       # Holochain conductor (agent-centric P2P runtime)
  hc              # Holochain dev CLI (scaffold/sandbox/pack/launch)
  lair-keystore   # Secure keystore for Holochain agent keys (cryptographic identity)
  nats-server     # NATS event bus for agent messaging (BUILDKIT L6)
]
