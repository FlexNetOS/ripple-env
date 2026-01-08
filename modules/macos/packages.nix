# macOS-specific packages
# Additional packages needed on Darwin systems
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
  home.packages = with pkgs; [
    # GNU coreutils (macOS alternatives)
    coreutils          # GNU core utilities
    findutils          # GNU find
    gnugrep            # GNU grep
    gnused             # GNU sed
    gawk               # GNU awk

    # macOS utilities
    darwin.trash       # Move to trash (rm alternative)

    # Build tools
    libiconv           # Character encoding conversion

    # Network tools
    iproute2mac        # iproute2 equivalent for macOS

    # Virtualization (for Linux containers)
    lima               # Linux VMs for container support

    # SSH utilities
    sshfs              # Mount remote filesystems
  ];

  # macOS-specific environment variables
  home.sessionVariables = {
    # Use Homebrew OpenSSL
    LDFLAGS = "-L/opt/homebrew/opt/openssl@3/lib";
    CPPFLAGS = "-I/opt/homebrew/opt/openssl@3/include";
    PKG_CONFIG_PATH = "/opt/homebrew/opt/openssl@3/lib/pkgconfig";

    # Fix for library loading on macOS
    DYLD_FALLBACK_LIBRARY_PATH = "$HOME/.nix-profile/lib:/usr/local/lib:/usr/lib";
  };

  # macOS-specific shell aliases
  home.shellAliases = {
    # Use GNU versions of commands
    "sed" = "gsed";
    "awk" = "gawk";

    # Open Finder
    "o" = "open";
    "oo" = "open .";

    # Flush DNS cache
    "flushdns" = "sudo dscacheutil -flushcache; sudo killall -HUP mDNSResponder";

    # Show/hide hidden files in Finder
    "showfiles" = "defaults write com.apple.finder AppleShowAllFiles YES; killall Finder";
    "hidefiles" = "defaults write com.apple.finder AppleShowAllFiles NO; killall Finder";
  };
}
