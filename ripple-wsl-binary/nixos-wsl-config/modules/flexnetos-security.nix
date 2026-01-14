{ config, lib, pkgs, ... }:

with lib;

{
  config = mkIf config.flexnetos.enable {
    # Security settings
    security = {
      sudo = {
        enable = true;
        wheelNeedsPassword = false;
      };
      
      pam.services.su.wheelOnly = true;
    };

    # Firewall (disabled for WSL - host handles this)
    networking.firewall.enable = false;

    # User security
    users.mutableUsers = false;
    
    # SSH hardening
    services.openssh.settings = {
      PasswordAuthentication = false;
      PermitRootLogin = "no";
      PubkeyAuthentication = true;
      MaxAuthTries = 3;
      LoginGraceTime = 60;
      MaxSessions = 3;
      ClientAliveInterval = 300;
      ClientAliveCountMax = 2;
    };

    # System hardening
    systemd.services.sshd.serviceConfig = {
      PrivateTmp = true;
      PrivateDevices = true;
      ProtectHome = true;
      ProtectSystem = "strict";
      NoNewPrivileges = true;
    };

    # File permissions
    systemd.tmpfiles.rules = [
      "Z /opt/flexnetos 755 flexnet users"
      "Z /var/lib/flexnetos 755 flexnet users"
      "Z /etc/flexnetos 755 root root"
      "Z /var/log/flexnetos 755 flexnet users"
    ];

    # Audit logging
    services.journald.extraConfig = ''
      Storage=persistent
      Compress=yes
      MaxRetentionSec=30day
      MaxFileSec=7day
      SystemMaxUse=1G
      SystemKeepFree=500M
      MaxLevelStore=info
      MaxLevelSyslog=info
      MaxLevelKMsg=notice
      MaxLevelConsole=info
    '';

    # Security packages
    environment.systemPackages = with pkgs; [
      fail2ban
      aide
      rkhunter
      chkrootkit
      logrotate
    ];

    # Fail2ban configuration
    services.fail2ban = {
      enable = true;
      settings = {
        DEFAULT = {
          bantime = 3600;
          findtime = 600;
          maxretry = 3;
        };
        
        sshd = {
          enabled = true;
          port = "ssh";
          filter = "sshd";
          logpath = "/var/log/auth.log";
          maxretry = 3;
        };
      };
    };

    # Log rotation
    services.logrotate.settings = {
      "/var/log/flexnetos/*.log" = {
        frequency = "daily";
        rotate = 7;
        compress = true;
        delaycompress = true;
        missingok = true;
        notifempty = true;
        create = "644 flexnet users";
      };
    };
  };
}
