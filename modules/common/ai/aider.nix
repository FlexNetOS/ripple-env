# Aider configuration
# AI pair programming with Git integration
#
# Features:
#   - Git-aware code editing
#   - Repo mapping for context
#   - Voice-to-code support
#   - Watch mode for auto-commits
#   - 100+ language support
#
# Usage:
#   pair                    # Start aider in current repo
#   pair src/               # Work on specific files
#   pair --voice            # Voice-to-code mode
#   pair --watch            # Auto-commit on file changes
#   aider --model claude    # Use specific model
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkEnableOption mkOption mkIf types;
in
{
  options.programs.aider = {
    enable = mkEnableOption "aider AI pair programming";

    package = mkOption {
      type = types.package;
      default = pkgs.aider-chat;
      description = "The aider-chat package to use";
    };

    settings = mkOption {
      type = types.attrs;
      default = { };
      description = "aider configuration options";
      example = {
        model = "claude-3-sonnet-20240229";
        auto-commits = true;
        dark-mode = true;
      };
    };
  };

  config = mkIf config.programs.aider.enable {
    home.packages = [
      config.programs.aider.package
      pkgs.portaudio  # For voice features
    ];

    # XDG-compliant configuration
    xdg.configFile."aider/.aider.conf.yml" = mkIf (config.programs.aider.settings != { }) {
      text = builtins.toJSON config.programs.aider.settings;
    };

    # Shell aliases for aider
    home.shellAliases = {
      "pair" = "aider";
      "pair-voice" = "aider --voice";
      "pair-watch" = "aider --watch";
      "pair-claude" = "aider --model claude-3-sonnet-20240229";
      "pair-gpt4" = "aider --model gpt-4-turbo";
    };
  };
}
