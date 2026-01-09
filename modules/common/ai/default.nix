# AI assistants module
# Provides AI-powered development tools
{ pkgs, lib, ... }:
{
  imports = [
    ./aichat.nix
    ./aider.nix
    ./agentgateway.nix
  ];

  # Agent Gateway disabled by default until hashes are obtained
  # Enable with: programs.agentgateway.enable = true;
  programs.agentgateway.enable = lib.mkDefault false;
}
