# Zoxide configuration
# Smart directory jumping
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkDefault;
in
{
  programs.zoxide = {
    enable = mkDefault true;

    # Enable for all shells
    enableBashIntegration = true;
    enableZshIntegration = true;
    enableNushellIntegration = true;

    # Custom options
    options = [
      "--cmd"
      "cd"
    ];
  };
}
