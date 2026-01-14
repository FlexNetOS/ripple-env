# Script Contract: isolate-cpu.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/isolate-cpu.sh`

---

## Purpose

CPU isolation utility for real-time robotics with PREEMPT_RT kernel support. Manages CPU isolation via kernel boot parameters (isolcpus, nohz_full, rcu_nocbs), IRQ affinity migration, CPU governor control, and taskset verification. Provides setup instructions for GRUB and NixOS configurations.

---

## Invocation

```bash
./scripts/isolate-cpu.sh COMMAND [ARGS]
```

**Commands:**
- `status` - Show current CPU isolation status
- `setup CPUS` - Generate kernel parameter setup (e.g., "2-7")
- `irq-migrate CPUS` - Migrate IRQs away from isolated CPUs
- `governor GOVERNOR [CPUS]` - Set CPU frequency governor (performance/powersave)
- `verify CPUS` - Verify isolation effectiveness
- `benchmark CPUS` - Run latency benchmark on isolated CPUs
- `help` - Show help

**Examples:**
```bash
./scripts/isolate-cpu.sh status
./scripts/isolate-cpu.sh setup "2-7"          # Isolate CPUs 2-7
./scripts/isolate-cpu.sh irq-migrate "2-7"    # Migrate IRQs
./scripts/isolate-cpu.sh governor performance "2-7"
./scripts/isolate-cpu.sh verify "2-7"
./scripts/isolate-cpu.sh benchmark "2-7"
```

---

## Side Effects

### IRQ Affinity (lines 138-168)
- Modifies `/proc/irq/*/smp_affinity` to move hardware interrupts away from isolated CPUs

### CPU Governor (lines 170-202)
- Writes to `/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor`
- Changes frequency scaling policy (performance/powersave)

### No Persistent Changes
- Kernel parameters require manual edit of `/etc/default/grub` or NixOS configuration
- Script only generates instructions, does not modify boot config

---

## Safety Classification

**🟡 CAUTION** - Modifies IRQ affinity and CPU governor (runtime only).

---

## Idempotency

**✅ FULLY IDEMPOTENT** - IRQ and governor changes can be reapplied.

---

## Key Features

### Kernel Parameter Generation (lines 112-136)

```bash
cmd_setup() {
    local cpus="${1:-}"

    if [[ -z "$cpus" ]]; then
        log_error "Usage: $0 setup CPUS"
        log_error "Example: $0 setup 2-7"
        return 1
    fi

    echo ""
    log_info "Kernel parameters for CPU isolation:"
    echo ""
    echo "  isolcpus=$cpus nohz_full=$cpus rcu_nocbs=$cpus"
    echo ""
    echo "For GRUB, edit /etc/default/grub:"
    echo "  GRUB_CMDLINE_LINUX=\"isolcpus=$cpus nohz_full=$cpus rcu_nocbs=$cpus\""
    echo ""
    echo "For NixOS, add to configuration.nix:"
    echo "  boot.kernelParams = [ \"isolcpus=$cpus\" \"nohz_full=$cpus\" \"rcu_nocbs=$cpus\" ];"
}
```

**Parameters explained:**
- `isolcpus` - Remove CPUs from kernel scheduler
- `nohz_full` - Disable periodic timer ticks (tickless mode)
- `rcu_nocbs` - Offload RCU callbacks to housekeeping CPUs

### IRQ Migration (lines 138-168)

```bash
cmd_irq_migrate() {
    local isolated_cpus="${1:-}"

    # Convert CPU list to affinity mask (invert)
    local num_cpus
    num_cpus=$(nproc)
    local affinity_mask="ffffffff"  # Default: all CPUs

    log_info "Migrating IRQs away from CPUs: $isolated_cpus"

    for irq in /proc/irq/*/; do
        if [[ -f "${irq}smp_affinity" ]]; then
            local irq_num
            irq_num=$(basename "$irq")
            echo "$affinity_mask" > "${irq}smp_affinity" 2>/dev/null || true
        fi
    done

    log_success "IRQ migration complete"
}
```

### Status Report (lines 72-110)

```bash
cmd_status() {
    echo ""
    log_info "CPU Isolation Status"
    echo "===================="

    # Check kernel parameters
    if [[ -f /proc/cmdline ]]; then
        local cmdline
        cmdline=$(cat /proc/cmdline)

        if echo "$cmdline" | grep -q "isolcpus="; then
            local isolated
            isolated=$(echo "$cmdline" | grep -o "isolcpus=[^ ]*" | cut -d= -f2)
            log_success "Isolated CPUs: $isolated"
        else
            log_warn "No isolcpus parameter found"
        fi
    fi

    # Check CPU governors
    for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
        if [[ -f "$cpu/cpufreq/scaling_governor" ]]; then
            local governor
            governor=$(cat "$cpu/cpufreq/scaling_governor")
            echo "  $(basename "$cpu"): $governor"
        fi
    done
}
```

### Verification (lines 204-240)

```bash
cmd_verify() {
    local cpus="${1:-}"

    log_info "Verifying CPU isolation for: $cpus"

    # Check if CPUs are truly isolated (no processes running)
    for cpu in $(echo "$cpus" | tr ',' ' ' | tr '-' ' '); do
        local tasks
        tasks=$(ps -eLo psr,comm | awk -v cpu="$cpu" '$1 == cpu {print $2}' | wc -l)

        if ((tasks <= 5)); then
            log_success "CPU $cpu: $tasks tasks (good isolation)"
        else
            log_warn "CPU $cpu: $tasks tasks (poor isolation)"
        fi
    done
}
```

### Benchmark (lines 242-280)

```bash
cmd_benchmark() {
    local cpus="${1:-}"

    if ! command -v cyclictest &>/dev/null; then
        log_error "cyclictest not found. Install rt-tests package."
        return 1
    fi

    log_info "Running latency benchmark on CPUs: $cpus"
    log_info "Duration: 60 seconds"

    # Run cyclictest with thread affinity
    local cpu_list
    cpu_list=$(echo "$cpus" | tr '-' ',')

    sudo cyclictest \
        --affinity="$cpu_list" \
        --priority=99 \
        --interval=1000 \
        --duration=60 \
        --quiet
}
```

**Requires:** `rt-tests` package for `cyclictest` utility.

---

## Dependencies

### Required Kernel Features
- PREEMPT_RT patch or CONFIG_PREEMPT_RT (fully preemptible kernel)
- CPU frequency scaling support (cpufreq)

### Optional Tools
- `cyclictest` (from rt-tests) - For latency benchmarking
- `bc` - For numeric calculations

### Permissions
- Root/sudo required for IRQ migration, governor changes, cyclictest

---

## References

- **Main script:** `scripts/isolate-cpu.sh` (287 lines)
- **Status command:** lines 72-110
- **Setup command:** lines 112-136
- **IRQ migration:** lines 138-168
- **Governor control:** lines 170-202
- **Verification:** lines 204-240
- **Benchmark:** lines 242-280
- **External:** [PREEMPT_RT Documentation](https://wiki.linuxfoundation.org/realtime/start)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 53/60 (88.3%)
