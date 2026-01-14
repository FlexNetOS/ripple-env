## Trivy config-scan exceptions

This repository runs a security gate (`make security`) that includes a Trivy **config** scan:

- `trivy fs --security-checks vuln,config .`

The config scan evaluates Kubernetes manifests against a broad ruleset. Some rules are:

1) environment/policy dependent (e.g., “images must come from a private registry”), or
2) incompatible with certain workloads by design (e.g., node-level monitoring DaemonSets), or
3) not applied in the **base** manifests to avoid breaking runtime behavior.

Those items are tracked in `.trivyignore` using Trivy's misconfiguration IDs.

### What we currently ignore (and why)

#### Private registry / trusted registry policy

- `AVD-KSV-0032`, `AVD-KSV-0033`, `AVD-KSV-0034`, `AVD-KSV-0035`, `AVD-KSV-0125`

These rules require images to be pulled only from private or explicitly-trusted registries. The base manifests reference upstream public images for ease of setup.

**Production recommendation:** mirror/pin images into your own registry (and ideally pin by digest) and remove these ignores.

#### UID/GID range policy

- `AVD-KSV-0020`, `AVD-KSV-0021`

These rules enforce a minimum UID/GID (e.g., > 10000). Many upstream images run as non-root but with UIDs below that threshold.

**Production recommendation:** use internally-built images that run as a known high UID/GID, or enforce this policy via an admission controller plus image hardening.

#### Netdata DaemonSet host access

- `AVD-KSV-0023`, `AVD-KSV-0024`, `AVD-KSV-0121`, `AVD-KSV-0005`, `AVD-KSV-0006`, `AVD-KSV-0009`, `AVD-KSV-0010`, `AVD-KSV-0012`, `AVD-KSV-0017`, `AVD-KSV-0022`

Netdata's Kubernetes daemonset commonly requires host-level access (e.g., `hostNetwork`, `hostPID`, host mounts of `/proc` and `/sys`, and sometimes host ports) to collect node metrics.

These rules also cover related, expected consequences of that design (privileged containers, added capabilities like `SYS_ADMIN`, and mounting `docker.sock`).

This is an **accepted risk** when deploying Netdata in a trusted cluster.

**Production recommendation:**

- deploy Netdata only on trusted nodes/namespaces
- restrict RBAC + network policies
- consider a managed/agentless observability alternative for hardened environments

#### RBAC node/proxy access

- `AVD-KSV-0047`

Some observability components use RBAC permissions that include node proxy access to reach node-level endpoints in certain setups.

**Production recommendation:** tighten RBAC to the minimal set required for your cluster and remove the ignore if you don't need node proxy.

#### Controller RBAC (Argo Rollouts)

- `AVD-KSV-0048`, `AVD-KSV-0056`

Progressive delivery controllers typically require permissions that look broad to generic RBAC scanners (e.g., patching Services/Ingresses for traffic shifting, and managing ReplicaSets). These permissions are part of the expected operational model.

**Production recommendation:** prefer the vendor-maintained manifests/Helm charts for Argo Rollouts, scope permissions as narrowly as your rollout strategy allows, and deploy into a locked-down namespace with additional admission controls.

#### Placeholder tokens in ConfigMaps

- `AVD-KSV-01010`

Netdata's config includes placeholder token fields. These must **not** be treated as real secrets.

**Production recommendation:** move real tokens into a `Secret` and mount it, or source them from an external secret manager.

#### ReadOnlyRootFilesystem for stateful apps

- `AVD-KSV-0014`

`readOnlyRootFilesystem: true` is a good default, but enabling it for certain stateful workloads (e.g., Postgres) or apps that write caches can require additional mounts (`emptyDir` for `/tmp`, `/var/run`, app caches) and careful validation.

The base manifests prioritize working defaults.

**Production recommendation:** apply a hardened overlay that enables read-only root filesystems and adds the required writable mounts.

### How to revisit exceptions

1. Remove an ID from `.trivyignore`.
2. Run `make security`.
3. Fix the underlying configuration or document a workload-specific exception.
