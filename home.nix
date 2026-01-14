{ config, pkgs, lib, ... }:

{
  # Let Home Manager install and manage itself
  programs.home-manager.enable = true;

  # Home Manager version (use stable for production)
  home.stateVersion = "24.11"; # Update when you upgrade Home Manager

  # User information - only use valid options
  home.username = builtins.getEnv "USER";
  home.homeDirectory = builtins.getEnv "HOME";

  # =============================================================================
  # Environment Variables for AI/ML Tools
  # =============================================================================
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

  # =============================================================================
  # Development Tools Configuration
  # =============================================================================
  
  # Git configuration
  programs.git = {
    enable = true;
    userName = "Ripple Environment User";
    userEmail = "user@example.com";
    extraConfig = {
      init.defaultBranch = "main";
      pull.rebase = true;
      push.autoSetupRemote = true;
      core.editor = "hx";
    };
  };

  # Helix editor
  programs.helix = {
    enable = true;
  };

  # =============================================================================
  # Package Management
  # =============================================================================
  
  # Install some useful packages
  home.packages = with pkgs; [
    # Terminal tools
    htop
    tree
    jq
    
    # Development tools
    git
    gh
    
    # Text processing
    ripgrep
    fd
  ];

  # =============================================================================
  # Shell Configuration
  # =============================================================================
  
  programs.bash = {
    enable = true;
    bashrcExtra = ''
      # Ripple environment aliases
      alias ripple-enter='source $HOME/ripple-env/scripts/stable-env.sh'
      alias pixi-safe='timeout 300 pixi'
      alias nix-safe='timeout 600 nix'
      alias docker-safe='timeout 120 docker'
      
      # Load ripple environment if available
      if [[ -f "$HOME/ripple-env/scripts/stable-env.sh" ]]; then
        source "$HOME/ripple-env/scripts/stable-env.sh"
      fi
    '';
  };

  # =============================================================================
  # Directory Creation
  # =============================================================================
  
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
}