# Bootstrap Script Testing Strategy

This document describes the multi-tier testing approach for validating the `bootstrap.ps1` and `bootstrap.sh` scripts.

## Overview

The testing strategy uses a 3-tier approach that balances coverage, speed, and cost:

| Tier | Type | Runner | WSL Support | Frequency |
|------|------|--------|-------------|-----------|
| 1 | Unit Tests | `windows-latest` / `ubuntu-latest` | Mocked | Every commit |
| 2 | Integration | `windows-latest` + setup-wsl | Ubuntu (limited) | Every commit |
| 3 | E2E | Self-hosted `[Windows, WSL2]` | Full NixOS | Manual |

## Tier 1: Unit Tests

**Location**: `test/unit/`

### PowerShell Tests (`bootstrap.tests.ps1`)

Tests PowerShell functions with mocked WSL2 calls. Runs on any Windows system without requiring actual WSL2.

```powershell
# Run locally
Invoke-Pester -Path test/unit/bootstrap.tests.ps1 -CI

# Run with coverage
Invoke-Pester -Path test/unit -CI -CodeCoverage bootstrap.ps1
```

**What's tested:**
- `Write-ColorOutput` - Output formatting
- `Test-Administrator` - Admin check
- `Test-WindowsVersion` - OS compatibility
- `Test-WSLInstalled` - WSL detection (mocked)
- `Test-WSL2Default` - WSL2 default check (mocked)
- `Test-DistroExists` - Distribution detection (mocked)
- `Set-WSLConfig` - Configuration file creation
- `Install-WSL` - WSL installation commands (mocked)
- `New-NixOSDistro` - Distribution import (mocked)

### Bash Validation

```bash
# Shellcheck validation
shellcheck bootstrap.sh

# Syntax validation
bash -n bootstrap.sh
```

## Tier 2: Integration Tests

**Location**: `test/integration/`

Uses the [Vampire/setup-wsl](https://github.com/Vampire/setup-wsl) GitHub Action to create a real WSL environment on GitHub-hosted runners.

### Limitations

The `setup-wsl` action provides:
- Real WSL2 kernel
- Ubuntu 22.04 distribution
- Basic WSL commands

It does **NOT** support:
- NixOS-WSL distribution
- Custom distribution import
- Full nested virtualization

### Integration Tests (`wsl-setup.tests.ps1`)

```powershell
# Run locally (requires WSL2)
Invoke-Pester -Path test/integration -Tag "Integration"

# Skip NixOS-specific tests
Invoke-Pester -Path test/integration -ExcludeTag "NixOS"
```

**What's tested:**
- WSL2 environment detection
- Basic Linux command execution
- Nix installation in WSL Ubuntu
- Bootstrap script syntax validation

## Tier 3: End-to-End Tests

**Runner**: Self-hosted with labels `[self-hosted, Windows, WSL2]`

Runs the complete bootstrap flow with NixOS-WSL on a real Windows machine with full WSL2 support.

### Setup Options

#### Option A: Local Windows Machine

Follow [self-hosted-runner-setup.md](./self-hosted-runner-setup.md)

#### Option B: Cloud Deployment (AWS)

See [infrastructure/README.md](../../infrastructure/README.md) for Terraform/Packer deployment.

```bash
# Deploy self-hosted runner to AWS
cd infrastructure/terraform
terraform init
terraform plan -var="github_token=YOUR_TOKEN"
terraform apply -var="github_token=YOUR_TOKEN"
```

### Triggering E2E Tests

E2E tests only run on manual `workflow_dispatch`:

1. Go to **Actions** > **Test Bootstrap Scripts**
2. Click **Run workflow**
3. The E2E job will run if a self-hosted runner is available

## Workflow Structure

```yaml
# .github/workflows/test-bootstrap.yml

jobs:
  unit-tests-powershell:    # Tier 1 - Always runs
    runs-on: windows-latest

  unit-tests-bash:          # Tier 1 - Always runs
    runs-on: ubuntu-latest

  integration-tests-wsl:    # Tier 2 - After unit tests
    runs-on: windows-latest
    needs: [unit-tests-powershell, unit-tests-bash]

  e2e-tests-self-hosted:    # Tier 3 - Manual only
    runs-on: [self-hosted, Windows, WSL2]
    needs: [unit-tests-powershell, unit-tests-bash]
    if: github.event_name == 'workflow_dispatch'
```

## Running Tests Locally

### Prerequisites

```powershell
# Install Pester (PowerShell 5.1+)
Install-Module Pester -Force -SkipPublisherCheck

# Install PSScriptAnalyzer
Install-Module PSScriptAnalyzer -Force -SkipPublisherCheck
```

### Run All Tests

```powershell
# Unit tests only (no WSL required)
Invoke-Pester -Path test/unit -CI

# Integration tests (requires WSL2)
Invoke-Pester -Path test/integration -CI

# All tests with verbosity
Invoke-Pester -Path test -Output Detailed
```

### Run with Test Results Output

```powershell
$config = New-PesterConfiguration
$config.Run.Path = "test"
$config.TestResult.Enabled = $true
$config.TestResult.OutputPath = "test-results.xml"
$config.TestResult.OutputFormat = "NUnitXml"
$config.Output.Verbosity = "Detailed"
Invoke-Pester -Configuration $config
```

## Adding New Tests

### Unit Test Structure

```powershell
Describe "Function-Name" {
    Context "When condition is met" {
        BeforeAll {
            Mock External-Command { return "mocked" }
        }

        It "Should do expected behavior" {
            $result = Function-Name
            $result | Should -Be "expected"
        }
    }
}
```

### Integration Test Structure

```powershell
Describe "Feature" -Tag "Integration" {
    BeforeAll {
        # Check if prerequisite is available
        $available = Test-Prerequisite
    }

    It "Should work" -Skip:(-not $available) {
        # Test that requires the prerequisite
    }
}
```

## CI/CD Integration

### Test Results

Test results are uploaded as artifacts:
- `unit-test-results` - Pester XML results from unit tests
- `integration-test-results` - Pester XML results from E2E tests

### Test Summary

The workflow generates a summary in `$GITHUB_STEP_SUMMARY`:
- Unit test status
- Bash validation status
- Integration test status

## Troubleshooting

### Unit Tests Failing

```powershell
# Check Pester version
Get-Module Pester | Select-Object Version

# Run with diagnostic output
Invoke-Pester -Path test/unit -Output Diagnostic
```

### Integration Tests Skipped

Integration tests are skipped when:
- WSL2 is not installed
- NixOS distribution doesn't exist
- Running on unsupported OS

### E2E Tests Not Running

1. Verify self-hosted runner is online
2. Check runner has labels: `self-hosted`, `Windows`, `WSL2`
3. Workflow must be triggered via `workflow_dispatch`

## Future Improvements

1. **Code Coverage** - Add coverage reports for bootstrap.ps1
2. **Performance Tests** - Measure bootstrap script execution time
3. **Matrix Testing** - Test across Windows 10/11 versions
4. **macOS Testing** - Add tests for bootstrap.sh on macOS
