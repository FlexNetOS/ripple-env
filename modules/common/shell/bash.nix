# Bash configuration
# Enhanced bash with modern defaults
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
  programs.bash = {
    enable = mkDefault true;

    # Enable completion
    enableCompletion = true;

    # History settings
    historyControl = [
      "erasedups"
      "ignorespace"
    ];
    historySize = 100000;
    historyFileSize = 100000;
    historyIgnore = [
      "ls"
      "cd"
      "exit"
      "clear"
    ];

    # Shell options
    shellOptions = [
      "autocd"
      "cdspell"
      "checkwinsize"
      "cmdhist"
      "expand_aliases"
      "extglob"
      "globstar"
      "histappend"
      "nocaseglob"
    ];

    # Init extra
    initExtra = ''
      # Better prompt (simple, fast)
      PS1='\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '

      # ROS2 environment function
      ros2-env() {
        env | grep ROS
      }

      # Quick colcon build
      cb() {
        colcon build --symlink-install "$@"
      }

      # Direnv hook
      eval "$(direnv hook bash)"

      # Zoxide hook
      eval "$(zoxide init bash)"

      # Carapace completions
      export CARAPACE_BRIDGES='zsh,fish,bash,inshellisense'
      source <(carapace _carapace)
    '';

    # Session variables
    sessionVariables = {
      EDITOR = "hx";
      VISUAL = "hx";
      LESS = "-R -F -X";
    };
  };
}
