# AI assistants module
# Provides AI-powered development tools
{ pkgs, lib, ... }:
{
  imports = [
    ./aichat.nix
    ./aider.nix
  ];
}
