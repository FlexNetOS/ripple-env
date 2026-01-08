# Zsh configuration
# Feature-rich shell with good defaults
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
  programs.zsh = {
    enable = mkDefault true;

    # Enable auto-cd
    autocd = true;

    # Enable autosuggestions
    autosuggestion.enable = true;

    # Enable syntax highlighting
    syntaxHighlighting.enable = true;

    # History settings
    history = {
      size = 100000;
      save = 100000;
      path = "$HOME/.zsh_history";
      ignoreDups = true;
      ignoreSpace = true;
      expireDuplicatesFirst = true;
      share = true;
    };

    # Completion settings
    enableCompletion = true;
    completionInit = ''
      autoload -U compinit && compinit
      zstyle ':completion:*' menu select
      zstyle ':completion:*' matcher-list 'm:{a-zA-Z}={A-Za-z}'
    '';

    # Init extra
    initExtra = ''
      # Vi mode
      bindkey -e  # Use emacs mode (more familiar for most)

      # Better history search
      bindkey '^R' history-incremental-search-backward
      bindkey '^S' history-incremental-search-forward

      # Word navigation
      bindkey '^[[1;5C' forward-word
      bindkey '^[[1;5D' backward-word

      # Home/End
      bindkey '^[[H' beginning-of-line
      bindkey '^[[F' end-of-line

      # Delete key
      bindkey '^[[3~' delete-char

      # ROS2 environment function
      ros2-env() {
        env | grep ROS
      }

      # Quick colcon build
      cb() {
        colcon build --symlink-install "$@"
      }

      # Direnv hook
      eval "$(direnv hook zsh)"

      # Zoxide hook
      eval "$(zoxide init zsh)"

      # Carapace completions
      export CARAPACE_BRIDGES='zsh,fish,bash,inshellisense'
      zstyle ':completion:*' format $'\e[2;37mCompleting %d\e[m'
      source <(carapace _carapace)
    '';

    # Session variables
    sessionVariables = {
      EDITOR = "hx";
      VISUAL = "hx";
    };
  };
}
