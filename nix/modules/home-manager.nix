# Home Manager Modules for ripple-env
# Provides common development tool configurations

{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.ripple.home-manager;
in
{
  options.ripple.home-manager = {
    enable = mkEnableOption "ripple home-manager modules";
    
    development-tools = {
      enable = mkEnableOption "development tool configurations" // { default = true; };
      
      # Editor configurations
      helix = {
        enable = mkEnableOption "helix editor configuration" // { default = true; };
        theme = mkOption {
          type = types.str;
          default = "onedark";
          description = "Helix editor theme";
        };
      };
      
      neovim = {
        enable = mkEnableOption "neovim configuration" // { default = false; };
      };
      
      # Shell configurations
      shell = {
        enable = mkEnableOption "shell configurations" // { default = true; };
        
        bash = {
          enable = mkEnableOption "bash configuration" // { default = true; };
          extraAliases = mkOption {
            type = types.attrsOf types.str;
            default = {};
            description = "Additional bash aliases";
          };
        };
        
        zsh = {
          enable = mkEnableOption "zsh configuration" // { default = true; };
          extraAliases = mkOption {
            type = types.attrsOf types.str;
            default = {};
            description = "Additional zsh aliases";
          };
        };
      };
      
      # Terminal tools
      terminal = {
        enable = mkEnableOption "terminal tool configurations" // { default = true; };
        
        tmux = {
          enable = mkEnableOption "tmux configuration" // { default = true; };
          shortcut = mkOption {
            type = types.str;
            default = "a";
            description = "Tmux prefix shortcut";
          };
        };
        
        fzf = {
          enable = mkEnableOption "fzf configuration" // { default = true; };
        };
        
        bat = {
          enable = mkEnableOption "bat configuration" // { default = true; };
          theme = mkOption {
            type = types.str;
            default = "TwoDark";
            description = "Bat syntax highlighting theme";
          };
        };
      };
    };
    
    # Ripple-specific configurations
    ripple-env = {
      enable = mkEnableOption "ripple environment configuration" // { default = true; };
      
      # Environment variables
      environment = mkOption {
        type = types.attrsOf types.str;
        default = {};
        description = "Additional environment variables";
      };
      
      # Aliases specific to ripple
      aliases = mkOption {
        type = types.attrsOf types.str;
        default = {};
        description = "Ripple-specific command aliases";
      };
    };
  };

  config = mkIf cfg.enable {
    # Development tools configuration
    programs = mkMerge [
      (mkIf cfg.development-tools.enable {
        # Helix editor
        helix = mkIf cfg.development-tools.helix.enable {
          enable = true;
          settings = {
            editor = {
              line-number = "relative";
              mouse = true;
              auto-save = true;
              auto-format = true;
              theme = cfg.development-tools.helix.theme;
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
        
        # Neovim
        neovim = mkIf cfg.development-tools.neovim.enable {
          enable = true;
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
        
        # Shell configurations
        bash = mkIf (cfg.development-tools.enable && cfg.development-tools.shell.enable && cfg.development-tools.shell.bash.enable) {
          enable = true;
          enableCompletion = true;
          bashrcExtra = ''
            # Ripple environment setup
            export RIPPLE_ENV_ROOT="${builtins.getEnv "PWD"}"
            export EDITOR="hx"
            export VISUAL="hx"
            
            # Base aliases
            alias ll='ls -alF'
            alias la='ls -A'
            alias l='ls -CF'
            alias gs='git status'
            alias gc='git commit'
            alias gp='git push'
            alias gl='git log --oneline --graph'
            
            # Custom aliases
            ${lib.concatStringsSep "\n" (lib.mapAttrsToList (k: v: "alias ${k}='${v}'") cfg.development-tools.shell.bash.extraAliases)}
          '';
        };
        
        zsh = mkIf (cfg.development-tools.enable && cfg.development-tools.shell.enable && cfg.development-tools.shell.zsh.enable) {
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
            
            # Base aliases
            alias ll='ls -alF'
            alias la='ls -A'
            alias l='ls -CF'
            alias gs='git status'
            alias gc='git commit'
            alias gp='git push'
            alias gl='git log --oneline --graph'
            
            # Custom aliases
            ${lib.concatStringsSep "\n" (lib.mapAttrsToList (k: v: "alias ${k}='${v}'") cfg.development-tools.shell.zsh.extraAliases)}
          '';
        };
        
        # Terminal tools
        tmux = mkIf (cfg.development-tools.enable && cfg.development-tools.terminal.enable && cfg.development-tools.terminal.tmux.enable) {
          enable = true;
          shortcut = cfg.development-tools.terminal.tmux.shortcut;
          
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
        
        fzf = mkIf (cfg.development-tools.enable && cfg.development-tools.terminal.enable && cfg.development-tools.terminal.fzf.enable) {
          enable = true;
          enableBashIntegration = true;
          enableZshIntegration = true;
        };
        
        bat = mkIf (cfg.development-tools.enable && cfg.development-tools.terminal.enable && cfg.development-tools.terminal.bat.enable) {
          enable = true;
          config = {
            theme = cfg.development-tools.terminal.bat.theme;
            style = "numbers,changes,header";
          };
        };
      })
      
      # Git configuration
      (mkIf cfg.development-tools.enable {
        git = {
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
      })
    ];
    
    # Ripple environment configuration
    home = mkIf cfg.ripple-env.enable {
      sessionVariables = cfg.ripple-env.environment // {
        RIPPLE_ENV_ROOT = builtins.getEnv "PWD";
      };
      
      file = {
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
    };
  };
}