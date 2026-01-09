# XDG Base Directory compliance configuration
# Ensures clean $HOME with standardized paths
# See: https://specifications.freedesktop.org/basedir/latest/
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkDefault mkOption types;
in
{
  # Enable XDG Base Directory Specification
  xdg.enable = mkDefault true;

  # User directories (Desktop, Documents, etc.)
  xdg.userDirs = {
    enable = mkDefault true;
    createDirectories = mkDefault true;
  };

  # Ensure XDG directories exist
  # Note: xdg.enable handles configHome, dataHome, cacheHome, stateHome

  # Configure XDG-compliant paths for common tools
  home.sessionVariables = {
    # Ensure XDG variables are available in non-login shells
    XDG_CONFIG_HOME = "${config.home.homeDirectory}/.config";
    XDG_DATA_HOME = "${config.home.homeDirectory}/.local/share";
    XDG_STATE_HOME = "${config.home.homeDirectory}/.local/state";
    XDG_CACHE_HOME = "${config.home.homeDirectory}/.cache";

    # Force XDG compliance for common tools
    # Cargo (Rust)
    CARGO_HOME = "${config.xdg.dataHome}/cargo";
    RUSTUP_HOME = "${config.xdg.dataHome}/rustup";

    # NPM/Node.js
    NPM_CONFIG_USERCONFIG = "${config.xdg.configHome}/npm/npmrc";
    NPM_CONFIG_CACHE = "${config.xdg.cacheHome}/npm";
    NODE_REPL_HISTORY = "${config.xdg.stateHome}/node_repl_history";

    # Python
    PYTHONSTARTUP = "${config.xdg.configHome}/python/pythonrc";
    PYTHON_HISTORY = "${config.xdg.stateHome}/python_history";
    IPYTHONDIR = "${config.xdg.configHome}/ipython";
    JUPYTER_CONFIG_DIR = "${config.xdg.configHome}/jupyter";

    # Docker
    DOCKER_CONFIG = "${config.xdg.configHome}/docker";

    # GNU tools
    HISTFILE = "${config.xdg.stateHome}/bash/history";
    LESSHISTFILE = "${config.xdg.stateHome}/less/history";

    # GnuPG
    GNUPGHOME = "${config.xdg.dataHome}/gnupg";

    # wget
    WGETRC = "${config.xdg.configHome}/wgetrc";

    # SQLite
    SQLITE_HISTORY = "${config.xdg.stateHome}/sqlite_history";

    # ROS2 (already XDG-compliant by default)
    # ROS_HOME defaults to ~/.ros which we could override to XDG
    # ROS_HOME = "${config.xdg.dataHome}/ros";
  };

  # Create necessary directories
  home.file = {
    # Ensure state directories exist for history files
    ".local/state/bash/.keep".text = "";
    ".local/state/less/.keep".text = "";

    # NPM config directory
    ".config/npm/.keep".text = "";

    # Python config (empty pythonrc to avoid errors)
    ".config/python/pythonrc".text = ''
      # Python startup file
      # XDG-compliant location
    '';
  };
}
