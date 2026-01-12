# Workflow Commands Module

This directory contains modular workflow command definitions for the ros2-humble-env development shell.

## Structure

```
nix/commands/workflow/
├── default.nix      # Aggregator - imports and re-exports all modules
├── github.nix       # GitHub CLI integration (gh-issues)
├── temporal.nix     # Temporal workflow management
├── n8n.nix          # n8n automation workflows
├── status.nix       # Combined status dashboard
├── sbom.nix         # SBOM generation and scanning
└── README.md        # This file
```

## Usage

### In flake.nix

```nix
# Import the workflow module
workflowCommands = import ./nix/commands/workflow { inherit pkgs; };

# Use all commands
packages = workflowCommands.all;

# Or selective imports
packages = workflowCommands.github ++ workflowCommands.status;
```

### Available Exports

| Export | Description | Commands |
|--------|-------------|----------|
| `all` | All workflow commands | All below |
| `github` | GitHub integration | `gh-issues` |
| `temporal` | Temporal workflows | `temporal-ctl` |
| `n8n` | n8n automation | `n8n-ctl` |
| `status` | Status dashboard | `workflow-status`, `open-url` |
| `sbom` | Supply chain security | `sbom-generate`, `sbom-scan`, `sbom-audit` |
| `minimal` | Status only | `workflow-status` |
| `githubTools` | GitHub + status | `gh-issues`, `workflow-status` |
| `orchestration` | Temporal + n8n + status | `temporal-ctl`, `n8n-ctl`, `workflow-status` |

## Commands Reference

### GitHub Commands (`github.nix`)

```bash
# List and manage GitHub issues
gh-issues list              # List open issues
gh-issues create            # Create new issue
gh-issues assign <n> <user> # Assign issue
```

### Temporal Commands (`temporal.nix`)

```bash
# Manage Temporal workflows
temporal-ctl start          # Start Temporal server
temporal-ctl stop           # Stop Temporal server
temporal-ctl status         # Check server status
temporal-ctl namespaces     # List namespaces
temporal-ctl workflows      # List running workflows
```

### n8n Commands (`n8n.nix`)

```bash
# Manage n8n automation
n8n-ctl start               # Start n8n server
n8n-ctl stop                # Stop n8n server
n8n-ctl status              # Check server status
n8n-ctl workflows           # List workflows
n8n-ctl export              # Export workflows
```

### Status Commands (`status.nix`)

```bash
# Combined status dashboard
workflow-status             # Show all service status
workflow-status --json      # Output as JSON

# URL helper
open-url <url>              # Open URL in browser
```

### SBOM Commands (`sbom.nix`)

```bash
# Supply chain security
sbom-generate [target] [output] [format]  # Generate SBOM
sbom-scan [sbom.json]                      # Scan for vulnerabilities
sbom-sign <artifact>                       # Sign with cosign
sbom-verify <artifact>                     # Verify signature
sbom-audit [target]                        # Full audit pipeline
```

## Adding New Commands

1. Create a new file `nix/commands/workflow/<name>.nix`:

```nix
{ pkgs, ... }:

with pkgs; [
  (writeShellScriptBin "my-command" ''
    #!/usr/bin/env bash
    echo "My new command"
  '')
]
```

2. Import it in `default.nix`:

```nix
let
  myModule = import ./my-module.nix { inherit pkgs; };
in
{
  myModule = myModule;
  all = github ++ temporal ++ n8n ++ status ++ sbom ++ myModule;
}
```

## Design Principles

1. **Modularity**: Each file is self-contained and independently testable
2. **Composability**: Modules can be combined in various ways
3. **Backward Compatibility**: The `all` export maintains previous behavior
4. **Documentation**: Each command should have `--help` support
5. **Error Handling**: Commands should fail gracefully with helpful messages

## Testing

```bash
# Enter development shell
nix develop

# Test individual commands
workflow-status --help
gh-issues --help
sbom-generate --help

# Test module import
nix eval .#devShells.x86_64-linux.default.packages
```
