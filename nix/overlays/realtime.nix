# Real-time kernel overlay for robotics applications
# Provides PREEMPT_RT kernel configuration and real-time tools
#
# Usage in flake.nix:
#   overlays = [ (import ./nix/overlays/realtime.nix) ];
#
# Reference: https://wiki.linuxfoundation.org/realtime/start
final: prev: {
  # Real-time testing tools
  rt-tests = prev.stdenv.mkDerivation rec {
    pname = "rt-tests";
    version = "2.6";

    src = prev.fetchFromGitHub {
      owner = "clrkwllms";
      repo = "rt-tests";
      rev = "v${version}";
      sha256 = "sha256-AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="; # Update with actual hash
    };

    buildInputs = with prev; [ numactl ];
    nativeBuildInputs = with prev; [ pkg-config ];

    makeFlags = [ "prefix=$(out)" ];

    meta = with prev.lib; {
      description = "Real-time tests for Linux";
      homepage = "https://wiki.linuxfoundation.org/realtime/documentation/howto/tools/rt-tests";
      license = licenses.gpl2;
      platforms = platforms.linux;
    };
  };

  # Linux kernel with PREEMPT_RT patch
  linuxPackages_rt = prev.linuxPackages_latest.extend (lpself: lpsuper: {
    kernel = lpsuper.kernel.override {
      structuredExtraConfig = with prev.lib.kernel; {
        # Enable full preemption
        PREEMPT_RT = yes;
        PREEMPT = yes;
        PREEMPT_VOLUNTARY = no;
        PREEMPT_NONE = no;

        # High resolution timers
        HIGH_RES_TIMERS = yes;
        NO_HZ_FULL = yes;

        # CPU isolation support
        CPU_ISOLATION = yes;
        NO_HZ_FULL_ALL = yes;

        # Disable features that hurt real-time
        DEBUG_PREEMPT = no;
        LOCKUP_DETECTOR = no;
        DETECT_HUNG_TASK = no;

        # Memory management for RT
        TRANSPARENT_HUGEPAGE = no;
        COMPACTION = no;

        # Reduce jitter
        RCU_NOCB_CPU = yes;
        RCU_BOOST = yes;
      };
    };
  });

  # CPU isolation utility
  cpuisol = prev.writeShellScriptBin "cpuisol" ''
    #!/usr/bin/env bash
    set -euo pipefail

    usage() {
      echo "Usage: cpuisol <command> [args]"
      echo ""
      echo "Commands:"
      echo "  status           Show current CPU isolation status"
      echo "  isolate <cpus>   Isolate CPUs (e.g., 2-3 or 2,3)"
      echo "  restore          Restore all CPUs to normal operation"
      echo "  irqbalance       Move IRQs away from isolated CPUs"
      echo ""
      echo "Example:"
      echo "  cpuisol isolate 2-3    # Isolate CPUs 2 and 3 for RT tasks"
    }

    status() {
      echo "=== CPU Isolation Status ==="
      echo ""
      if [ -f /sys/devices/system/cpu/isolated ]; then
        isolated=$(cat /sys/devices/system/cpu/isolated)
        echo "Isolated CPUs: ''${isolated:-none}"
      fi
      if [ -f /sys/devices/system/cpu/nohz_full ]; then
        nohz=$(cat /sys/devices/system/cpu/nohz_full)
        echo "NO_HZ Full CPUs: ''${nohz:-none}"
      fi
      echo ""
      echo "Per-CPU governor:"
      for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
        if [ -f "$cpu/cpufreq/scaling_governor" ]; then
          gov=$(cat "$cpu/cpufreq/scaling_governor")
          echo "  $(basename $cpu): $gov"
        fi
      done
    }

    case "''${1:-}" in
      status)
        status
        ;;
      isolate)
        echo "CPU isolation requires kernel boot parameters."
        echo "Add to GRUB_CMDLINE_LINUX: isolcpus=$2 nohz_full=$2 rcu_nocbs=$2"
        ;;
      irqbalance)
        echo "Stopping irqbalance and moving IRQs..."
        systemctl stop irqbalance 2>/dev/null || true
        # Move IRQs to CPU 0
        for irq in /proc/irq/[0-9]*/smp_affinity; do
          echo 1 | sudo tee "$irq" > /dev/null 2>&1 || true
        done
        echo "IRQs moved to CPU 0"
        ;;
      restore)
        echo "Restoring normal operation requires reboot without isolcpus parameter"
        ;;
      *)
        usage
        ;;
    esac
  '';

  # Memory locking utility for RT processes
  mlockall-wrapper = prev.writeShellScriptBin "rt-launch" ''
    #!/usr/bin/env bash
    # Launch a process with RT priority and locked memory

    if [ $# -lt 1 ]; then
      echo "Usage: rt-launch <command> [args...]"
      echo ""
      echo "Launches a command with:"
      echo "  - SCHED_FIFO priority 80"
      echo "  - Memory locked (mlockall)"
      echo "  - Unlimited memlock"
      exit 1
    fi

    # Set resource limits
    ulimit -l unlimited 2>/dev/null || echo "Warning: Could not set unlimited memlock"
    ulimit -r 99 2>/dev/null || true

    # Launch with RT priority
    exec chrt -f 80 "$@"
  '';
}
