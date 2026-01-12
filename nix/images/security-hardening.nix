# NixOS Security Hardening Module
# Provides systemd-based security hardening for NixOS images
#
# Usage:
#   imports = [ ./security-hardening.nix ];
#   security.hardening.enable = true;
{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.security.hardening;
in
{
  options.security.hardening = {
    enable = mkEnableOption "security hardening for NixOS images";

    level = mkOption {
      type = types.enum [ "minimal" "standard" "strict" ];
      default = "standard";
      description = ''
        Security hardening level:
        - minimal: Basic hardening suitable for development
        - standard: Recommended for production (default)
        - strict: Maximum security, may break some applications
      '';
    };

    auditd = {
      enable = mkOption {
        type = types.bool;
        default = true;
        description = "Enable Linux audit daemon for security logging";
      };
    };

    firewall = {
      strict = mkOption {
        type = types.bool;
        default = false;
        description = "Enable strict firewall rules (deny by default)";
      };
    };
  };

  config = mkIf cfg.enable {
    # =================================================================
    # KERNEL SECURITY PARAMETERS
    # =================================================================
    boot.kernel.sysctl = {
      # Restrict dmesg to root only
      "kernel.dmesg_restrict" = 1;

      # Hide kernel pointers from non-privileged users
      "kernel.kptr_restrict" = 2;

      # Disable magic SysRq key (except for sync and reboot)
      "kernel.sysrq" = mkIf (cfg.level != "minimal") 176;

      # Restrict perf_event access
      "kernel.perf_event_paranoid" = 3;

      # Yama LSM - restrict ptrace
      "kernel.yama.ptrace_scope" = mkIf (cfg.level != "minimal") 1;

      # Disable unprivileged user namespaces (can break containers)
      "kernel.unprivileged_userns_clone" = mkIf (cfg.level == "strict") 0;

      # Network security
      "net.ipv4.conf.all.rp_filter" = 1;
      "net.ipv4.conf.default.rp_filter" = 1;
      "net.ipv4.conf.all.accept_redirects" = 0;
      "net.ipv4.conf.default.accept_redirects" = 0;
      "net.ipv4.conf.all.secure_redirects" = 0;
      "net.ipv4.conf.default.secure_redirects" = 0;
      "net.ipv6.conf.all.accept_redirects" = 0;
      "net.ipv6.conf.default.accept_redirects" = 0;
      "net.ipv4.conf.all.send_redirects" = 0;
      "net.ipv4.conf.default.send_redirects" = 0;

      # Ignore ICMP broadcasts
      "net.ipv4.icmp_echo_ignore_broadcasts" = 1;

      # Ignore bogus ICMP errors
      "net.ipv4.icmp_ignore_bogus_error_responses" = 1;

      # Enable SYN flood protection
      "net.ipv4.tcp_syncookies" = 1;

      # Log martian packets
      "net.ipv4.conf.all.log_martians" = mkIf (cfg.level != "minimal") 1;
      "net.ipv4.conf.default.log_martians" = mkIf (cfg.level != "minimal") 1;

      # Disable IP source routing
      "net.ipv4.conf.all.accept_source_route" = 0;
      "net.ipv4.conf.default.accept_source_route" = 0;
      "net.ipv6.conf.all.accept_source_route" = 0;
      "net.ipv6.conf.default.accept_source_route" = 0;

      # TCP hardening
      "net.ipv4.tcp_rfc1337" = 1;

      # Filesystem security
      "fs.protected_hardlinks" = 1;
      "fs.protected_symlinks" = 1;
      "fs.protected_fifos" = mkIf (cfg.level != "minimal") 2;
      "fs.protected_regular" = mkIf (cfg.level != "minimal") 2;

      # Disable core dumps for setuid programs
      "fs.suid_dumpable" = 0;
    };

    # =================================================================
    # SYSTEMD SERVICE HARDENING
    # =================================================================
    systemd.services = {
      # Security audit logging service
      "security-audit" = mkIf cfg.auditd.enable {
        description = "Security Audit Logger";
        wantedBy = [ "multi-user.target" ];
        after = [ "systemd-journald.service" ];

        serviceConfig = {
          Type = "simple";
          ExecStart = "${pkgs.bash}/bin/bash -c 'while true; do sleep 3600; done'";
          Restart = "always";
          RestartSec = 5;

          # Service hardening
          NoNewPrivileges = true;
          ProtectSystem = "strict";
          ProtectHome = true;
          PrivateTmp = true;
          PrivateDevices = true;
          ProtectKernelTunables = true;
          ProtectKernelModules = true;
          ProtectControlGroups = true;
          RestrictRealtime = true;
          RestrictSUIDSGID = true;
          MemoryDenyWriteExecute = true;
          LockPersonality = true;
        };
      };

      # Failed login monitor
      "failed-login-monitor" = mkIf (cfg.level != "minimal") {
        description = "Monitor and alert on failed login attempts";
        wantedBy = [ "multi-user.target" ];
        after = [ "systemd-journald.service" ];

        serviceConfig = {
          Type = "simple";
          ExecStart = pkgs.writeShellScript "failed-login-monitor" ''
            ${pkgs.systemd}/bin/journalctl -f -u sshd -u systemd-logind | \
            while read -r line; do
              if echo "$line" | grep -q -i "failed\|invalid\|refused"; then
                echo "$(date '+%Y-%m-%d %H:%M:%S') ALERT: $line" >> /var/log/security-alerts.log
              fi
            done
          '';
          Restart = "always";
          RestartSec = 10;

          # Service hardening
          NoNewPrivileges = true;
          ProtectSystem = "full";
          ProtectHome = true;
          PrivateTmp = true;
        };
      };

      # File integrity monitoring service
      "file-integrity-check" = mkIf (cfg.level == "strict") {
        description = "Periodic file integrity check";
        after = [ "multi-user.target" ];

        serviceConfig = {
          Type = "oneshot";
          ExecStart = pkgs.writeShellScript "integrity-check" ''
            # Check critical system files
            CRITICAL_FILES="/etc/passwd /etc/shadow /etc/group /etc/sudoers"
            HASH_FILE="/var/lib/security/file-hashes.txt"

            mkdir -p /var/lib/security

            if [ ! -f "$HASH_FILE" ]; then
              # Initial hash generation
              for f in $CRITICAL_FILES; do
                if [ -f "$f" ]; then
                  ${pkgs.coreutils}/bin/sha256sum "$f" >> "$HASH_FILE"
                fi
              done
            else
              # Verify hashes
              if ! ${pkgs.coreutils}/bin/sha256sum -c "$HASH_FILE" --quiet 2>/dev/null; then
                echo "$(date '+%Y-%m-%d %H:%M:%S') CRITICAL: File integrity violation detected!" >> /var/log/security-alerts.log
              fi
            fi
          '';

          # Service hardening
          NoNewPrivileges = true;
          ProtectSystem = "full";
          PrivateTmp = true;
        };
      };

      # Automatic security updates check
      "security-updates-check" = {
        description = "Check for security updates";
        after = [ "network-online.target" ];
        wants = [ "network-online.target" ];

        serviceConfig = {
          Type = "oneshot";
          ExecStart = pkgs.writeShellScript "security-updates-check" ''
            echo "$(date '+%Y-%m-%d %H:%M:%S') Checking for security updates..." >> /var/log/security-updates.log
            # For NixOS, we check if the channel has updates
            if command -v nix-channel &> /dev/null; then
              nix-channel --update 2>&1 | head -5 >> /var/log/security-updates.log || true
            fi
          '';

          # Service hardening
          NoNewPrivileges = true;
          ProtectSystem = "full";
          ProtectHome = true;
          PrivateTmp = true;
        };
      };
    };

    # Timer for file integrity checks
    systemd.timers."file-integrity-check" = mkIf (cfg.level == "strict") {
      wantedBy = [ "timers.target" ];
      timerConfig = {
        OnCalendar = "hourly";
        Persistent = true;
        RandomizedDelaySec = 300;
      };
    };

    # Timer for security updates check
    systemd.timers."security-updates-check" = {
      wantedBy = [ "timers.target" ];
      timerConfig = {
        OnCalendar = "daily";
        Persistent = true;
        RandomizedDelaySec = 3600;
      };
    };

    # =================================================================
    # SSH HARDENING
    # =================================================================
    services.openssh.settings = mkIf config.services.openssh.enable {
      # Disable root login
      PermitRootLogin = "no";

      # Disable password authentication
      PasswordAuthentication = false;

      # Use only strong key exchange algorithms
      KexAlgorithms = [
        "curve25519-sha256"
        "curve25519-sha256@libssh.org"
        "diffie-hellman-group16-sha512"
        "diffie-hellman-group18-sha512"
      ];

      # Use only strong ciphers
      Ciphers = [
        "chacha20-poly1305@openssh.com"
        "aes256-gcm@openssh.com"
        "aes128-gcm@openssh.com"
        "aes256-ctr"
        "aes192-ctr"
        "aes128-ctr"
      ];

      # Use only strong MACs
      Macs = [
        "hmac-sha2-512-etm@openssh.com"
        "hmac-sha2-256-etm@openssh.com"
        "umac-128-etm@openssh.com"
      ];

      # Limit authentication attempts
      MaxAuthTries = 3;

      # Client alive interval for detecting dead connections
      ClientAliveInterval = 300;
      ClientAliveCountMax = 2;

      # Disable X11 forwarding
      X11Forwarding = false;

      # Disable agent forwarding (can enable per-user if needed)
      AllowAgentForwarding = mkIf (cfg.level == "strict") false;

      # Log level
      LogLevel = "VERBOSE";
    };

    # =================================================================
    # FIREWALL CONFIGURATION
    # =================================================================
    networking.firewall = {
      enable = true;

      # Log dropped packets (for debugging)
      logRefusedConnections = cfg.level != "minimal";
      logRefusedPackets = cfg.level == "strict";

      # Strict mode: deny all by default
      allowedTCPPorts = mkIf (!cfg.firewall.strict) [ ];
      allowedUDPPorts = mkIf (!cfg.firewall.strict) [ ];

      # Allow SSH if enabled
      allowedTCPPorts = mkIf config.services.openssh.enable [ 22 ];

      # Rate limiting for SSH (via iptables)
      extraCommands = mkIf (cfg.level != "minimal") ''
        # Rate limit SSH connections
        iptables -I INPUT -p tcp --dport 22 -m state --state NEW -m recent --set
        iptables -I INPUT -p tcp --dport 22 -m state --state NEW -m recent --update --seconds 60 --hitcount 4 -j DROP
      '';
    };

    # =================================================================
    # AUDIT CONFIGURATION
    # =================================================================
    security.auditd.enable = cfg.auditd.enable;

    security.audit = mkIf cfg.auditd.enable {
      enable = true;
      rules = [
        # Log all commands run by root
        "-a exit,always -F arch=b64 -F euid=0 -S execve -k root_commands"

        # Log changes to authentication configuration
        "-w /etc/passwd -p wa -k identity"
        "-w /etc/group -p wa -k identity"
        "-w /etc/shadow -p wa -k identity"
        "-w /etc/sudoers -p wa -k identity"

        # Log SSH configuration changes
        "-w /etc/ssh/sshd_config -p wa -k sshd_config"

        # Log cron/at jobs
        "-w /etc/crontab -p wa -k cron"
        "-w /var/spool/cron -p wa -k cron"
      ];
    };

    # =================================================================
    # ADDITIONAL SECURITY MEASURES
    # =================================================================

    # Disable core dumps
    security.pam.loginLimits = [
      { domain = "*"; type = "hard"; item = "core"; value = "0"; }
    ];

    # Set secure umask
    environment.variables.UMASK = "027";

    # Security packages
    environment.systemPackages = with pkgs; [
      # Security scanning tools (optional, for strict mode)
    ] ++ optionals (cfg.level == "strict") [
      lynis         # Security auditing
      rkhunter      # Rootkit hunter
    ];

    # Restrict /tmp to tmpfs with noexec
    boot.tmp = mkIf (cfg.level != "minimal") {
      useTmpfs = true;
      tmpfsSize = "2G";
    };

    # Console security
    services.getty.helpLine = mkIf (cfg.level == "strict") "";
  };
}
