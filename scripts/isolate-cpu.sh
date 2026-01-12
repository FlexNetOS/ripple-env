#!/usr/bin/env bash
# CPU Isolation Script for Real-Time Robotics
#
# Isolates specified CPUs for exclusive use by real-time processes.
# This reduces jitter and improves determinism for ROS2 nodes.
#
# Usage:
#   ./isolate-cpu.sh status          Show current isolation status
#   ./isolate-cpu.sh setup 2-3       Generate kernel parameters for CPUs 2-3
#   ./isolate-cpu.sh irq-migrate     Move IRQs away from isolated CPUs
#   ./isolate-cpu.sh governor perf   Set CPU governor to performance
#   ./isolate-cpu.sh verify          Verify RT setup
#
# Reference: https://www.kernel.org/doc/html/latest/admin-guide/kernel-parameters.html

set -euo pipefail

SCRIPT_NAME=$(basename "$0")

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

usage() {
    cat << EOF
Usage: $SCRIPT_NAME <command> [args]

Commands:
  status              Show current CPU isolation and RT status
  setup <cpus>        Generate kernel boot parameters for CPU isolation
  irq-migrate         Move IRQs to CPU 0 (requires root)
  governor <mode>     Set CPU frequency governor (performance/powersave)
  verify              Verify real-time kernel and configuration
  benchmark           Run quick latency benchmark with cyclictest

Examples:
  $SCRIPT_NAME status
  $SCRIPT_NAME setup 2-3          # Isolate CPUs 2 and 3
  $SCRIPT_NAME setup 2,3,4,5      # Isolate CPUs 2, 3, 4, 5
  $SCRIPT_NAME governor performance
  $SCRIPT_NAME benchmark

Notes:
  - CPU isolation requires kernel boot parameters (reboot needed)
  - IRQ migration and governor changes take effect immediately
  - Benchmark requires cyclictest (from rt-tests package)
EOF
}

cmd_status() {
    echo "=== CPU Isolation Status ==="
    echo ""

    # Check isolated CPUs
    if [ -f /sys/devices/system/cpu/isolated ]; then
        isolated=$(cat /sys/devices/system/cpu/isolated)
        if [ -n "$isolated" ]; then
            log_info "Isolated CPUs: $isolated"
        else
            log_warn "No CPUs isolated"
        fi
    fi

    # Check nohz_full
    if [ -f /sys/devices/system/cpu/nohz_full ]; then
        nohz=$(cat /sys/devices/system/cpu/nohz_full)
        if [ -n "$nohz" ]; then
            log_info "NO_HZ Full CPUs: $nohz"
        fi
    fi

    echo ""
    echo "=== Kernel Configuration ==="

    # Check for PREEMPT_RT
    if grep -q "PREEMPT_RT" /boot/config-$(uname -r) 2>/dev/null || \
       uname -v | grep -q "PREEMPT_RT"; then
        log_info "PREEMPT_RT: Enabled"
    else
        log_warn "PREEMPT_RT: Not detected (standard kernel)"
    fi

    # Check high-res timers
    if [ -f /proc/timer_list ]; then
        if grep -q "hres_active.*: 1" /proc/timer_list; then
            log_info "High-res timers: Active"
        fi
    fi

    echo ""
    echo "=== CPU Frequency Governors ==="
    for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
        if [ -f "$cpu/cpufreq/scaling_governor" ]; then
            gov=$(cat "$cpu/cpufreq/scaling_governor")
            freq=$(cat "$cpu/cpufreq/scaling_cur_freq" 2>/dev/null || echo "unknown")
            echo "  $(basename "$cpu"): $gov (${freq}kHz)"
        fi
    done

    echo ""
    echo "=== Memory Lock Limits ==="
    echo "  Soft: $(ulimit -Sl) kB"
    echo "  Hard: $(ulimit -Hl) kB"
}

cmd_setup() {
    local cpus="${1:-}"
    if [ -z "$cpus" ]; then
        log_error "Please specify CPUs to isolate (e.g., 2-3 or 2,3,4)"
        exit 1
    fi

    echo "=== Kernel Boot Parameters for CPU Isolation ==="
    echo ""
    echo "Add these parameters to your bootloader (GRUB, systemd-boot, etc.):"
    echo ""
    echo "  isolcpus=$cpus nohz_full=$cpus rcu_nocbs=$cpus"
    echo ""
    echo "For GRUB, edit /etc/default/grub:"
    echo "  GRUB_CMDLINE_LINUX=\"isolcpus=$cpus nohz_full=$cpus rcu_nocbs=$cpus\""
    echo ""
    echo "Then run:"
    echo "  sudo update-grub    # Debian/Ubuntu"
    echo "  sudo grub2-mkconfig -o /boot/grub2/grub.cfg  # Fedora/RHEL"
    echo ""
    echo "For NixOS, add to configuration.nix:"
    echo "  boot.kernelParams = [ \"isolcpus=$cpus\" \"nohz_full=$cpus\" \"rcu_nocbs=$cpus\" ];"
    echo ""
    log_warn "Reboot required for changes to take effect"
}

cmd_irq_migrate() {
    if [ "$(id -u)" -ne 0 ]; then
        log_error "IRQ migration requires root privileges"
        exit 1
    fi

    log_info "Stopping irqbalance service..."
    systemctl stop irqbalance 2>/dev/null || true
    systemctl disable irqbalance 2>/dev/null || true

    log_info "Moving all IRQs to CPU 0..."
    local moved=0
    for irq in /proc/irq/[0-9]*/smp_affinity; do
        if echo 1 > "$irq" 2>/dev/null; then
            ((moved++)) || true
        fi
    done

    log_info "Moved $moved IRQs to CPU 0"
    log_info "IRQ migration complete"
}

cmd_governor() {
    local mode="${1:-performance}"

    if [ "$(id -u)" -ne 0 ]; then
        log_error "Governor change requires root privileges"
        exit 1
    fi

    log_info "Setting CPU governor to: $mode"
    for cpu in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do
        if [ -f "$cpu" ]; then
            echo "$mode" > "$cpu" 2>/dev/null || true
        fi
    done

    # Disable turbo boost for consistency (optional)
    if [ -f /sys/devices/system/cpu/intel_pstate/no_turbo ]; then
        echo 1 > /sys/devices/system/cpu/intel_pstate/no_turbo 2>/dev/null || true
        log_info "Disabled Intel Turbo Boost for consistent frequency"
    fi

    log_info "Governor set to $mode on all CPUs"
}

cmd_verify() {
    echo "=== Real-Time Setup Verification ==="
    echo ""

    local issues=0

    # Check kernel
    if uname -v | grep -q "PREEMPT_RT" || grep -q "PREEMPT_RT=y" /boot/config-$(uname -r) 2>/dev/null; then
        log_info "PREEMPT_RT kernel: OK"
    else
        log_warn "PREEMPT_RT kernel: Not detected"
        ((issues++)) || true
    fi

    # Check CPU isolation
    isolated=$(cat /sys/devices/system/cpu/isolated 2>/dev/null || echo "")
    if [ -n "$isolated" ]; then
        log_info "CPU isolation: OK ($isolated)"
    else
        log_warn "CPU isolation: Not configured"
        ((issues++)) || true
    fi

    # Check governor
    gov=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "unknown")
    if [ "$gov" = "performance" ]; then
        log_info "CPU governor: OK (performance)"
    else
        log_warn "CPU governor: $gov (recommend: performance)"
        ((issues++)) || true
    fi

    # Check memlock limits
    memlimit=$(ulimit -l)
    if [ "$memlimit" = "unlimited" ] || [ "$memlimit" -gt 1048576 ]; then
        log_info "Memory lock limit: OK ($memlimit)"
    else
        log_warn "Memory lock limit: Low ($memlimit kB)"
        ((issues++)) || true
    fi

    echo ""
    if [ $issues -eq 0 ]; then
        log_info "All checks passed!"
    else
        log_warn "$issues issue(s) found - see recommendations above"
    fi
}

cmd_benchmark() {
    if ! command -v cyclictest &> /dev/null; then
        log_error "cyclictest not found. Install rt-tests package."
        exit 1
    fi

    log_info "Running quick latency benchmark (30 seconds)..."
    echo "Press Ctrl+C to stop early"
    echo ""

    # Run cyclictest with common settings
    sudo cyclictest \
        --mlockall \
        --priority=80 \
        --interval=1000 \
        --distance=0 \
        --duration=30 \
        --histogram=100 \
        --histfile=/tmp/cyclictest_hist.txt \
        2>&1 | tee /tmp/cyclictest_output.txt

    echo ""
    log_info "Results saved to /tmp/cyclictest_output.txt"
    log_info "Histogram saved to /tmp/cyclictest_hist.txt"
}

# Main
case "${1:-}" in
    status)
        cmd_status
        ;;
    setup)
        cmd_setup "${2:-}"
        ;;
    irq-migrate|irq)
        cmd_irq_migrate
        ;;
    governor|gov)
        cmd_governor "${2:-performance}"
        ;;
    verify|check)
        cmd_verify
        ;;
    benchmark|bench)
        cmd_benchmark
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        usage
        exit 1
        ;;
esac
