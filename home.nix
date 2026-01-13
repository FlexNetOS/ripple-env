# Home Manager configuration for ripple-env
# This file manages your user environment and dotfiles
#
# Usage:
#   1. Copy this file to ~/.config/home-manager/home.nix
#   2. Edit to customize your environment
#   3. Run: home-manager switch
#
# For more options, see: https://nix-community.github.io/home-manager/options.html

{ config, pkgs, lib, ... }:

{
  # Let Home Manager install and manage itself
  programs.home-manager.enable = true;

  # Home Manager version (use stable for production)
  home.stateVersion = "24.11"; # Update when you upgrade Home Manager

  # User information
  home.username = builtins.getEnv "USER";
  home.homeDirectory = builtins.getEnv "HOME";
  home.userFullName = "Ripple Environment User";
  home.userEmail = "user@example.com";

  # =============================================================================
  # Development Tools Configuration
  # =============================================================================
  
  # Shell configuration
  programs.bash = {
    enable = true;
    enableCompletion = true;
    bashrcExtra = ''
      # Ripple environment setup
      export RIPPLE_ENV_ROOT="${builtins.getEnv "PWD"}"
      export EDITOR="hx"
      export VISUAL="hx"
      
      # Aliases
      alias ll='ls -alF'
      alias la='ls -A'
      alias l='ls -CF'
      alias gs='git status'
      alias gc='git commit'
      alias gp='git push'
      alias gl='git log --oneline --graph'
    '';
  };

  programs.zsh = {
    enable = true;
    enableCompletion = true;
    enableAutosuggestions = true;
    syntaxHighlighting.enable = true;
    history.size = 10000;
    initExtra = ''
      # Ripple environment setup
      export RIPPLE_ENV_ROOT="${builtins.getEnv "PWD"}"
      export EDITOR="hx"
      export VISUAL="hx"
      
      # Aliases
      alias ll='ls -alF'
      alias la='ls -A'
      alias l='ls -CF'
      alias gs='git status'
      alias gc='git commit'
      alias gp='git push'
      alias gl='git log --oneline --graph'
    '';
  };

  # Git configuration
  programs.git = {
    enable = true;
    userName = "Ripple User";
    userEmail = "user@example.com";
    
    extraConfig = {
      init.defaultBranch = "main";
      pull.rebase = true;
      push.autoSetupRemote = true;
      core.editor = "hx";
      
      # Ripple-specific aliases
      ripple-log = "log --oneline --graph --decorate";
      ripple-status = "status --short --branch";
    };
  };

  # =============================================================================
  # AI/ML Environment Variables and Configuration
  # =============================================================================

  # Environment variables for AI/ML tools
  home.sessionVariables = {
    # Default editor (Helix)
    EDITOR = "${pkgs.helix}/bin/hx";
    VISUAL = "${pkgs.helix}/bin/hx";

    # LocalAI models directory
    LOCALAI_MODELS_PATH = "${config.home.homeDirectory}/.local/share/localai/models";

    # AIOS Agent OS
    AIOS_DIR = "${config.home.homeDirectory}/.local/share/aios";
    AIOS_PORT = "8000";

    # PromptCache
    PROMPTCACHE_DIR = "${config.home.homeDirectory}/.local/share/prompt-cache";
    PROMPTCACHE_PORT = "8080";

    # Ripple environment
    RIPPLE_ENV_ROOT = "${config.home.homeDirectory}/ripple-env";
    RIPPLE_ENV_CACHE = "${config.home.homeDirectory}/.cache/ripple-env";
    RIPPLE_ENV_CONFIG = "${config.home.homeDirectory}/.config/ripple-env";
  };

  # Create necessary directories
  home.file."${config.home.sessionVariables.LOCALAI_MODELS_PATH}" = {
    source = pkgs.emptyDirectory;
    recursive = true;
  };

  home.file."${config.home.sessionVariables.AIOS_DIR}" = {
    source = pkgs.emptyDirectory;
    recursive = true;
  };

  home.file."${config.home.sessionVariables.PROMPTCACHE_DIR}" = {
    source = pkgs.emptyDirectory;
    recursive = true;
  };

  home.file."${config.home.sessionVariables.RIPPLE_ENV_CACHE}" = {
    source = pkgs.emptyDirectory;
    recursive = true;
  };

  home.file."${config.home.sessionVariables.RIPPLE_ENV_CONFIG}" = {
    source = pkgs.emptyDirectory;
    recursive = true;
  };

  # =============================================================================
  # Editor Configuration
  # =============================================================================
  
  # Helix editor (primary)
  programs.helix = {
    enable = true;
    settings = {
      editor = {
        line-number = "relative";
        mouse = true;
        auto-save = true;
        auto-format = true;
      };
      
      keys.normal = {
        space = {
          q = ":w";
          Q = ":wq";
          f = ":format";
          F = ":lsp-format";
        };
      };
    };
  };

  # Neovim (alternative)
  programs.neovim = {
    enable = false; # Set to true if you prefer Neovim
    defaultEditor = false;
    viAlias = true;
    vimAlias = true;
    
    extraConfig = ''
      " Ripple environment specific settings
      set number
      set relativenumber
      set mouse=a
      set autoindent
      set expandtab
      set shiftwidth=2
      set softtabstop=2
    '';
  };

  # =============================================================================
  # Terminal & Shell Tools
  # =============================================================================
  
  programs.tmux = {
    enable = true;
    shortcut = "a";
    
    extraConfig = ''
      # Ripple-specific tmux config
      set -g default-terminal "screen-256color"
      set -g history-limit 10000
      set -g mouse on
      
      # Status bar
      set -g status-bg colour235
      set -g status-fg colour250
      set -g status-left '#[fg=colour39]#H #[fg=colour250]:: #[fg=colour39]#(whoami)'
      set -g status-right '#[fg=colour39]ripple-env #[fg=colour250]:: #[fg=colour39]#(date +%H:%M)'
    '';
  };

  programs.fzf = {
    enable = true;
    enableBashIntegration = true;
    enableZshIntegration = true;
  };

  programs.bat = {
    enable = true;
    config = {
      theme = "TwoDark";
      style = "numbers,changes,header";
    };
  };

  programs.eza = {
    enable = true;
    enableBashIntegration = true;
    enableZshIntegration = true;
    extraOptions = [
      "--group-directories-first"
      "--header"
      "--git"
    ];
  };

  # =============================================================================
  # Development Environment
  # =============================================================================
  
  # Node.js development
  programs.nodejs = {
    enable = true;
    package = pkgs.nodejs_22;
  };

  # Python development
  programs.python3 = {
    enable = true;
    package = pkgs.python313;
    
    # User Python packages (separate from project dependencies)
    userPackages = with pkgs.python313Packages; [
      pip
      virtualenv
      ipython
      jupyter
      black
      flake8
      mypy
    ];
  };

  # Rust development
  programs.rust = {
    enable = true;
    package = pkgs.rustc;
  };

  # =============================================================================
  # System Configuration
  # =============================================================================
  
  # XDG directories
  xdg.enable = true;
  
  # User services
  services = {
    # GPG agent
    gpg-agent = {
      enable = true;
      defaultCacheTtl = 1800;
      maxCacheTtl = 3600;
    };
    
    # SSH agent
    ssh-agent = {
      enable = true;
    };
  };

  # =============================================================================
  # Dotfile Management
  # =============================================================================
  
  # Create useful symlinks
  home.file = {
    # Ripple environment configuration
    ".config/ripple/env".text = ''
      RIPPLE_ENV_ROOT="${builtins.getEnv "PWD"}"
      RIPPLE_OFFLINE=0
    '';
    
    # Git ignore global
    ".config/git/ignore".text = ''
      # Ripple environment ignores
      .pixi/
      pixi.lock
      .envrc.local
      
      # Python
      __pycache__/
      *.py[cod]
      *$py.class
      
      # Nix
      result
      result-*
      
      # IDE
      .vscode/
      .idea/
      *.swp
      *.swo
      *~
    '';
  };

  # =============================================================================
  # Packages to install
  # =============================================================================
  
  # Additional packages not covered by program modules
  home.packages = with pkgs; [
    # Development tools
    just              # Command runner
    fd                # Fast find alternative
    ripgrep           # Fast grep alternative
    sd                # Intuitive sed alternative
    tokei             # Code statistics
    hyperfine         # Command-line benchmarking
    
    # System tools
    htop              # Process viewer
    btop              # Modern top alternative
    ncdu              # Disk usage analyzer
    tree              # Directory tree viewer
    
    # Text processing
    sd                # Search and replace
    ripgrep           # Fast grep
    fd                # Fast find
    
    # Network tools
    curl
    wget
    httpie            # Modern HTTP client
    
    # Archive tools
    zip
    unzip
    p7zip
    tar
    
    # Version control
    git
    gh                # GitHub CLI
    tig               # Text-mode interface for git
    
    # Containers
    docker-compose
    
    # Nix tools
    nix-output-monitor
    nix-tree
    nixfmt-rfc-style
    nil
  ];

  # =============================================================================
  # Activation Scripts
  # =============================================================================
  
  home.activation = {
    # Ensure ripple-env directory structure
    ensureRippleDirs = lib.hm.dag.entryAfter ["writeBoundary"] ''
      echo "Setting up ripple-env directories..."
      mkdir -p "$HOME/.local/share/ripple-env"
      mkdir -p "$HOME/.config/ripple-env"
      mkdir -p "$HOME/.cache/ripple-env"
    '';
    
    # Display welcome message
    showWelcome = lib.hm.dag.entryAfter ["ensureRippleDirs"] ''
      echo ""
      echo "🌊 Welcome to ripple-env!"
      echo "   Home Manager configuration activated."
      echo "   Edit ~/.config/home-manager/home.nix to customize."
      echo ""
    '';
  };
}