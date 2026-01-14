# Script Contract: init-docker-networks.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/init-docker-networks.sh`

---

## Purpose

Create isolated Docker networks for the ARIA platform with proper segmentation for security boundaries. Creates 5 networks with pre-defined IP ranges, conflict detection, and label management.

---

## Invocation

```bash
./scripts/init-docker-networks.sh [--clean|--remove|--status|--help]
```

**Options:**
- (none) - Create networks if not exists
- `--clean` - Remove and recreate all networks
- `--remove` - Remove all ARIA networks
- `--status` - Show current network status
- `--help` - Show help message

---

## Inputs

### Networks Created
| Network | Subnet | IP Range | Purpose |
|---------|--------|----------|---------|
| identity-network | 172.20.0.0/16 | 172.20.1.0/24 | Identity, authentication, secrets |
| agentic-network | 172.21.0.0/16 | 172.21.1.0/24 | AI/ML services, automation |
| observability-network | 172.22.0.0/16 | 172.22.1.0/24 | Monitoring, logging, tracing |
| data-network | 172.23.0.0/16 | 172.23.1.0/24 | Databases, storage |
| messaging-network | 172.24.0.0/16 | 172.24.1.0/24 | NATS, event streaming |

**Alternative ranges** (172.25-172.29) available for conflict resolution (lines 39-45).

**Evidence:** lines 30-54

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success - all networks created or removed |
| `1` | Failure - Docker unavailable or network creation failed |

---

## Side Effects

**Creates Docker networks** with:
- Driver: bridge
- Custom subnets and IP ranges
- ARIA-specific labels (`aria.network.purpose`, `aria.network.security`)
- **Evidence:** lines 131-137

**May remove networks** if --clean or --remove specified.

---

## Safety Classification

**🟢 SAFE** - Network management only, no data loss.

**Note:** --remove will fail if containers using networks (lines 154-158).

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Checks network existence before creation (lines 125-128).

---

## Conflict Detection

**Function:** `check_subnet_conflict(subnet, network_name)`
**Evidence:** lines 73-90

Checks:
1. **Docker networks** - Inspects all networks for subnet conflicts
2. **Host routing table** - Checks `ip route show` for subnet usage

---

## Key Functions

### create_network(name)
**Evidence:** lines 120-140
- Checks if network exists
- Creates with custom subnet, IP range, labels
- Uses bridge driver, file storage

### remove_network(name)
**Evidence:** lines 142-163
- Checks for connected containers
- Fails if containers still attached
- Removes network

### show_status()
**Evidence:** lines 165-189
- Lists all ARIA networks
- Shows subnet, purpose, connected containers
- Color-coded (✓ exists, ✗ missing)

---

## Labels Applied

**Evidence:** lines 135-136

```bash
--label "aria.network.purpose=${DESCRIPTIONS[$name]}"
--label "aria.network.security=segmented"
```

---

## References

### Source Code
- **Main script:** `scripts/init-docker-networks.sh` (331 lines)
- **Network definitions:** lines 30-54
- **Conflict detection:** lines 73-90
- **Network creation:** lines 120-140
- **Status display:** lines 165-189

### Related Files
- **Deployment scripts:** All deploy-*.sh scripts rely on these networks

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 10/60 contracts complete
