{ config, pkgs, wslConfig, ... }:

{
  imports = [
    # NixOS-WSL specific modules
    <nixos-wsl/modules>
    
    # FlexNetOS modules
    ./modules/flexnetos-core.nix
    ./modules/flexnetos-services.nix
    ./modules/flexnetos-security.nix
    ./modules/flexnetos-monitoring.nix
  ];

  # System basics
  system.stateVersion = "24.05";
  
  # WSL configuration
  wsl = wslConfig.wsl;

  # Boot configuration
  boot.kernel.sysctl = {
    "net.ipv4.ip_forward" = 1;
    "net.ipv6.conf.all.forwarding" = 1;
  };

  # Networking
  networking = {
    hostName = "flexnetos";
    useDHCP = false;
    firewall.enable = false; # WSL handles this
  };

  # Users
  users.users.flexnet = {
    isNormalUser = true;
    description = "FlexNetOS Administrator";
    extraGroups = [ "wheel" "docker" "systemd-journal" ];
    shell = pkgs.zsh;
    openssh.authorizedKeys.keys = [];
  };

  # Timezone
  time.timeZone = "UTC";

  # Locale
  i18n.defaultLocale = "en_US.UTF-8";

  # System packages
  environment.systemPackages = with pkgs; [
    # Core utilities
    git
    curl
    wget
    jq
    yq
    tmux
    htop
    vim
    neovim
    
    # Docker and container tools
    docker
    docker-compose
    kubernetes-helm
    k9s
    
    # Development tools
    gcc
    gnumake
    cmake
    pkg-config
    
    # FlexNetOS specific
    rustc
    cargo
    go_1_21
    nodejs_20
    python311
    python311Packages.pip
    
    # System utilities
    systemd
    networkmanager
    iptables
    
    # Monitoring
    prometheus
    grafana
    node_exporter
    
    # Security
    vault
    openssl
    gnupg
    fail2ban
    aide
  ];

  # Enable systemd
  systemd.enable = true;

  # Services
  services = {
    docker = {
      enable = true;
      autoPrune.enable = true;
    };

    openssh = {
      enable = true;
      settings.PermitRootLogin = "no";
      settings.PasswordAuthentication = false;
    };

    # Enable required kernel modules
    kernel-modules = {
      enable = true;
      modules = [ "br_netfilter" "overlay" "nf_conntrack" ];
    };
  };

  # Environment variables
  environment.variables = {
    FLEXNETOS_HOME = "/opt/flexnetos";
    FLEXNETOS_CONFIG = "/etc/flexnetos";
    FLEXNETOS_DATA = "/var/lib/flexnetos";
  };

  # System paths
  environment.pathsToLink = [ "/etc/flexnetos" ];

  # File systems
  fileSystems."/mnt/c" = {
    device = "C:";
    fsType = "drvfs";
    options = [ "metadata" ];
  };

  # Ensure directories exist
  systemd.tmpfiles.rules = [
    "d /opt/flexnetos 755 flexnet users -"
    "d /etc/flexnetos 755 root root -"
    "d /var/lib/flexnetos 755 flexnet users -"
    "d /var/log/flexnetos 755 flexnet users -"
  ];
}
