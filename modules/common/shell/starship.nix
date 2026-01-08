# Starship prompt configuration
# Cross-shell customizable prompt
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
  programs.starship = {
    enable = mkDefault true;

    # Enable for all shells
    enableBashIntegration = true;
    enableZshIntegration = true;
    enableNushellIntegration = true;

    # Starship configuration
    settings = {
      # Minimal, fast prompt
      format = lib.concatStrings [
        "$directory"
        "$git_branch"
        "$git_status"
        "$nix_shell"
        "$python"
        "$rust"
        "$cmd_duration"
        "$line_break"
        "$character"
      ];

      # Right prompt
      right_format = lib.concatStrings [
        "$time"
      ];

      # Prompt character
      character = {
        success_symbol = "[➜](bold green)";
        error_symbol = "[➜](bold red)";
      };

      # Directory
      directory = {
        truncation_length = 3;
        truncate_to_repo = true;
        style = "bold cyan";
      };

      # Git branch
      git_branch = {
        format = "[$symbol$branch]($style) ";
        style = "bold purple";
        symbol = " ";
      };

      # Git status
      git_status = {
        format = "([$all_status$ahead_behind]($style))";
        style = "bold red";
        conflicted = "=";
        ahead = "⇡$count";
        behind = "⇣$count";
        diverged = "⇕⇡$ahead_count⇣$behind_count";
        untracked = "?$count";
        stashed = "\\$$count";
        modified = "!$count";
        staged = "+$count";
        renamed = "»$count";
        deleted = "✘$count";
      };

      # Nix shell indicator
      nix_shell = {
        format = "[$symbol$state]($style) ";
        symbol = " ";
        style = "bold blue";
        pure_msg = "";
        impure_msg = "";
      };

      # Python version
      python = {
        format = "[$symbol$version]($style) ";
        symbol = " ";
        style = "yellow";
      };

      # Rust version
      rust = {
        format = "[$symbol$version]($style) ";
        symbol = " ";
        style = "red";
      };

      # Command duration
      cmd_duration = {
        min_time = 2000;
        format = "[$duration]($style) ";
        style = "yellow";
      };

      # Time
      time = {
        disabled = false;
        format = "[$time]($style)";
        style = "dimmed white";
        time_format = "%H:%M";
      };

      # Disable modules we don't need
      aws.disabled = true;
      azure.disabled = true;
      gcloud.disabled = true;
      kubernetes.disabled = true;
      docker_context.disabled = true;
    };
  };
}
