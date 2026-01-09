# AI assistants module
# Provides AI-powered development tools
{ pkgs, lib, ... }:
{
  imports = [
    ./aichat.nix
    ./aider.nix
    ./agentgateway.nix
    ./genai-toolbox.nix
  ];

  # Agent Gateway disabled by default until hashes are obtained
  # Enable with: programs.agentgateway.enable = true;
  programs.agentgateway.enable = lib.mkDefault false;

  # MCP Toolbox (genai-toolbox) for Layer 8 tool execution
  # Enable with: programs.genai-toolbox.enable = true;
  programs.genai-toolbox.enable = lib.mkDefault true;
}
