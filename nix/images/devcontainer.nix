# Devcontainer NixOS Image Configuration
# Builds a Docker image for use as a VSCode devcontainer
#
# Build command:
#   nix build .#devcontainer
#
# Load into Docker:
#   docker load < result
#
# Or use in devcontainer.json:
#   "image": "ripple-env-devcontainer:latest"
#
# This provides a fully reproducible Nix-based development environment
# that matches the WSL and local development setups.
{ pkgs, lib, ... }:

let
  # Core system packages (matches wsl.nix)
  systemPackages = with pkgs; [
    # Core utilities
    coreutils
    bashInteractive
    gnugrep
    gnused
    gawk
    findutils
    which
    less
    procps
    gnutar
    gzip
    xz
    cacert
    openssl
  ];

  # Development tools
  devTools = with pkgs; [
    # Git and VCS
    git
    git-lfs
    gh
    jujutsu
    lazygit

    # Core CLI tools
    curl
    wget
    jq
    yq
    ripgrep
    fd
    bat
    eza
    htop
    btop

    # Nix development
    direnv
    nix-direnv
    nix-output-monitor
    nix-tree
    nixfmt-rfc-style
    nil
    cachix

    # Shells
    zsh
    nushell
    starship
    zoxide

    # Editors
    helix
    neovim

    # Python
    python313
    python313Packages.pip
    python313Packages.virtualenv

    # Build tools
    gcc
    gnumake
    cmake
    pkg-config

    # Kubernetes
    kubectl
    kubernetes-helm
    kustomize
    k9s
    kubectx
  ];

  # All packages combined
  allPackages = systemPackages ++ devTools;

  # User setup script
  userSetupScript = pkgs.writeShellScript "user-setup" ''
    # Create user directories
    mkdir -p $HOME/.config/nushell
    mkdir -p $HOME/.config/direnv
    mkdir -p $HOME/.local/bin
    mkdir -p $HOME/.pixi/bin

    # Zsh configuration
    cat > $HOME/.zshrc << 'EOF'
    export PATH="/nix/var/nix/profiles/default/bin:$HOME/.pixi/bin:$HOME/.local/bin:$PATH"
    eval "$(direnv hook zsh)"
    eval "$(starship init zsh)"
    eval "$(zoxide init zsh)"
    export EDITOR=hx VISUAL=hx

    # Aliases
    alias ll="eza -la"
    alias la="eza -a"
    alias l="eza -l"
    alias cat="bat"
    EOF

    # Bash configuration
    cat > $HOME/.bashrc << 'EOF'
    export PATH="/nix/var/nix/profiles/default/bin:$HOME/.pixi/bin:$HOME/.local/bin:$PATH"
    eval "$(direnv hook bash)"
    eval "$(starship init bash)"
    eval "$(zoxide init bash)"
    export EDITOR=hx VISUAL=hx

    # Aliases
    alias ll="eza -la"
    alias la="eza -a"
    alias l="eza -l"
    alias cat="bat"
    EOF

    # Nushell configuration
    cat > $HOME/.config/nushell/env.nu << 'EOF'
    $env.PATH = ($env.PATH | prepend "/nix/var/nix/profiles/default/bin" | prepend $"($env.HOME)/.pixi/bin" | prepend $"($env.HOME)/.local/bin")
    $env.EDITOR = "hx"
    $env.VISUAL = "hx"
    EOF

    cat > $HOME/.config/nushell/config.nu << 'EOF'
    $env.config = { show_banner: false }
    alias ll = ls -l
    alias la = ls -la
    EOF

    # Direnv whitelist
    cat > $HOME/.config/direnv/direnv.toml << 'EOF'
    [whitelist]
    prefix = ["/workspaces", "/home"]
    EOF

    # Git LFS
    git lfs install --skip-repo 2>/dev/null || true
  '';

in
pkgs.dockerTools.buildLayeredImage {
  name = "ripple-env-devcontainer";
  tag = "latest";

  contents = allPackages ++ [
    # Add CA certificates for HTTPS
    pkgs.cacert

    # Add shadow for user management (needed by devcontainer)
    pkgs.shadow

    # Add sudo
    pkgs.sudo
  ];

  config = {
    Env = [
      "PATH=/nix/var/nix/profiles/default/bin:/root/.pixi/bin:/root/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
      "NIX_PATH=nixpkgs=channel:nixpkgs-unstable"
      "SSL_CERT_FILE=${pkgs.cacert}/etc/ssl/certs/ca-bundle.crt"
      "NIX_SSL_CERT_FILE=${pkgs.cacert}/etc/ssl/certs/ca-bundle.crt"
      "EDITOR=hx"
      "VISUAL=hx"
      "SHELL=/nix/var/nix/profiles/default/bin/zsh"
    ];
    WorkingDir = "/workspaces";
    Cmd = [ "${pkgs.zsh}/bin/zsh" ];
    User = "root";
  };

  # Extra commands to run during image build
  extraCommands = ''
    # Create necessary directories
    mkdir -p tmp
    chmod 1777 tmp

    mkdir -p etc
    mkdir -p root

    # Create /etc/passwd and /etc/group for user management
    echo "root:x:0:0:root:/root:/nix/var/nix/profiles/default/bin/zsh" > etc/passwd
    echo "root:x:0:" > etc/group

    # Create nix directories
    mkdir -p nix/var/nix/profiles

    # Symlink for SSL certificates
    mkdir -p etc/ssl/certs
    ln -sf ${pkgs.cacert}/etc/ssl/certs/ca-bundle.crt etc/ssl/certs/ca-certificates.crt
  '';

  # Max layers for efficient caching
  maxLayers = 100;
}
