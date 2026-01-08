# macOS shell configuration
# Additional shell setup specific to Darwin
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkDefault;
  # Detect architecture: Apple Silicon uses /opt/homebrew, Intel uses /usr/local
  isAarch64 = pkgs.stdenv.hostPlatform.isAarch64;
  homebrewPrefix = if isAarch64 then "/opt/homebrew" else "/usr/local";
in
{
  # macOS-specific shell configuration

  # Fix for nix-darwin shell environment
  home.file.".config/nushell/macos.nu" = {
    text = ''
      # macOS-specific Nushell configuration

      # Homebrew path handling (supports both Apple Silicon and Intel)
      # Apple Silicon: /opt/homebrew, Intel: /usr/local
      if ("/opt/homebrew/bin" | path exists) {
        $env.PATH = ($env.PATH | prepend "/opt/homebrew/bin")
        $env.PATH = ($env.PATH | prepend "/opt/homebrew/sbin")
      } else if ("/usr/local/bin" | path exists) {
        $env.PATH = ($env.PATH | prepend "/usr/local/bin")
        $env.PATH = ($env.PATH | prepend "/usr/local/sbin")
      }

      # Fix for DYLD on macOS
      if ($env.DYLD_FALLBACK_LIBRARY_PATH? | is-empty) {
        $env.DYLD_FALLBACK_LIBRARY_PATH = $"($env.HOME)/.nix-profile/lib:/usr/local/lib:/usr/lib"
      }

      # macOS-specific aliases
      alias o = open
      alias oo = open .
      alias trash = darwin.trash

      # Quick access to common directories
      def cdl [] { cd ~/Downloads }
      def cdd [] { cd ~/Desktop }
      def cdc [] { cd ~/Code }

      # System info
      def sysinfo [] {
        print $"(ansi green)macOS System Info(ansi reset)"
        print "=================="
        print $"Hostname: (sys).host.hostname"
        print $"OS: (sys).host.long_os_version"
        print $"Uptime: (sys).host.uptime"
        print $"Memory: (sys).mem.used / (sys).mem.total"
      }
    '';
  };

  # zsh macOS config
  programs.zsh.initExtra = mkDefault ''
    # macOS-specific zsh configuration

    # Homebrew completions (supports both Apple Silicon and Intel)
    if [[ -d /opt/homebrew/share/zsh/site-functions ]]; then
      FPATH="/opt/homebrew/share/zsh/site-functions:$FPATH"
    elif [[ -d /usr/local/share/zsh/site-functions ]]; then
      FPATH="/usr/local/share/zsh/site-functions:$FPATH"
    fi

    # Fix for slow paste in zsh
    zstyle ':bracketed-paste-magic' active-widgets '.self-*'

    # iTerm2 shell integration (if available)
    if [[ -f "$HOME/.iterm2_shell_integration.zsh" ]]; then
      source "$HOME/.iterm2_shell_integration.zsh"
    fi
  '';

  # bash macOS config
  programs.bash.initExtra = mkDefault ''
    # macOS-specific bash configuration

    # Homebrew completions (supports both Apple Silicon and Intel)
    if [[ -r /opt/homebrew/etc/profile.d/bash_completion.sh ]]; then
      source /opt/homebrew/etc/profile.d/bash_completion.sh
    elif [[ -r /usr/local/etc/profile.d/bash_completion.sh ]]; then
      source /usr/local/etc/profile.d/bash_completion.sh
    fi
  '';
}
