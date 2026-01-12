# Real-Time Kernel Configuration for ROS2
# NixOS module for PREEMPT_RT kernel and real-time system configuration
#
# Usage in configuration.nix:
#   imports = [ ./nix/os/realtime-kernel.nix ];
#   services.ros2-realtime.enable = true;
#   services.ros2-realtime.isolatedCores = [ 2 3 ];
{ config, lib, pkgs, ... }:

with lib;

let
  cfg = config.services.ros2-realtime;
in
{
  options.services.ros2-realtime = {
    enable = mkEnableOption "ROS2 real-time kernel configuration";

    isolatedCores = mkOption {
      type = types.listOf types.int;
      default = [];
      description = "CPU cores to isolate for real-time tasks";
      example = [ 2 3 ];
    };

    rtPriority = mkOption {
      type = types.int;
      default = 80;
      description = "Default RT priority for ROS2 processes";
    };

    memoryLock = mkOption {
      type = types.bool;
      default = true;
      description = "Allow RT processes to lock memory";
    };

    networkTuning = mkOption {
      type = types.bool;
      default = true;
      description = "Apply network optimizations for DDS";
    };
  };

  config = mkIf cfg.enable {
    # Use RT kernel
    boot.kernelPackages = pkgs.linuxPackages_rt;

    # Kernel parameters for real-time
    boot.kernelParams = mkIf (cfg.isolatedCores != []) [
      "isolcpus=${concatMapStringsSep "," toString cfg.isolatedCores}"
      "nohz_full=${concatMapStringsSep "," toString cfg.isolatedCores}"
      "rcu_nocbs=${concatMapStringsSep "," toString cfg.isolatedCores}"
    ];

    # Real-time scheduler settings
    boot.kernel.sysctl = {
      # Disable RT throttling (allow 100% CPU for RT tasks)
      "kernel.sched_rt_runtime_us" = -1;

      # Reduce swap usage for more predictable memory
      "vm.swappiness" = 10;

      # Disable NMI watchdog (reduces latency)
      "kernel.nmi_watchdog" = 0;

      # Timer slack (reduce for lower latency)
      "kernel.timer_slack_ns" = 1;
    } // optionalAttrs cfg.networkTuning {
      # Network optimizations for DDS
      "net.core.rmem_max" = 16777216;
      "net.core.wmem_max" = 16777216;
      "net.core.rmem_default" = 8388608;
      "net.core.wmem_default" = 8388608;
      "net.core.netdev_max_backlog" = 5000;
      "net.ipv4.tcp_low_latency" = 1;
      "net.ipv4.igmp_max_memberships" = 1024;
    };

    # CPU frequency governor for consistent performance
    powerManagement.cpuFreqGovernor = "performance";

    # Real-time group and permissions
    users.groups.realtime = {};

    security.pam.loginLimits = mkIf cfg.memoryLock [
      { domain = "@realtime"; type = "-"; item = "rtprio"; value = "99"; }
      { domain = "@realtime"; type = "-"; item = "memlock"; value = "unlimited"; }
      { domain = "@realtime"; type = "-"; item = "nice"; value = "-20"; }
    ];

    # Install RT testing tools
    environment.systemPackages = with pkgs; [
      linuxPackages_rt.cpupower
      numactl
      hwloc
      stress-ng
    ] ++ optionals (pkgs ? rt-tests) [
      rt-tests  # cyclictest, etc.
    ];

    # Systemd RT configuration
    systemd.extraConfig = ''
      CPUAffinity=${concatMapStringsSep " " toString cfg.isolatedCores}
      DefaultCPUAccounting=yes
      DefaultMemoryAccounting=yes
    '';

    # udev rules for real-time scheduling
    services.udev.extraRules = ''
      # Allow realtime group to use RT scheduling
      KERNEL=="cpu_dma_latency", GROUP="realtime", MODE="0660"
    '';
  };
}
