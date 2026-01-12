# Self-Hosted Windows Runner Setup for WSL2 Testing

This guide explains how to set up a self-hosted GitHub Actions runner on Windows with WSL2 for end-to-end testing of the `bootstrap.ps1` script.

## Requirements

- Windows 10 version 2004+ (Build 19041+) or Windows 11
- 16GB+ RAM recommended
- 100GB+ free disk space
- Administrator access
- GitHub repository admin access (to add runners)

## Step 1: Enable WSL2

Run PowerShell as Administrator:

```powershell
# Enable WSL
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart

# Enable Virtual Machine Platform
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart

# Restart your computer
Restart-Computer
```

After restart:

```powershell
# Set WSL2 as default
wsl --set-default-version 2

# Verify WSL2 is working
wsl --status
```

## Step 2: Install GitHub Actions Runner

1. Go to your repository on GitHub
2. Navigate to **Settings** → **Actions** → **Runners**
3. Click **New self-hosted runner**
4. Select **Windows** and **x64**
5. Follow the instructions to download and configure the runner

Example commands (update with your actual token):

```powershell
# Create a folder for the runner
mkdir C:\actions-runner
cd C:\actions-runner

# Download the latest runner package
Invoke-WebRequest -Uri https://github.com/actions/runner/releases/download/v2.311.0/actions-runner-win-x64-2.311.0.zip -OutFile actions-runner-win-x64.zip

# Extract the installer
Add-Type -AssemblyName System.IO.Compression.FileSystem
[System.IO.Compression.ZipFile]::ExtractToDirectory("$PWD\actions-runner-win-x64.zip", "$PWD")

# Configure the runner (use your repo URL)
.\config.cmd --url https://github.com/<OWNER>/<REPO> --token YOUR_TOKEN_HERE
```

## Step 3: Add Required Labels

When configuring the runner, add these labels:
- `self-hosted`
- `Windows`
- `WSL2`

Or add them via GitHub UI after registration:
1. Go to **Settings** → **Actions** → **Runners**
2. Click on your runner
3. Add labels: `Windows`, `WSL2`

## Step 4: Configure Runner as Service

Run PowerShell as Administrator:

```powershell
cd C:\actions-runner

# Install as a Windows service
.\svc.cmd install

# Start the service
.\svc.cmd start

# Check status
.\svc.cmd status
```

## Step 5: Configure WSL Memory Limits (Optional)

Create or edit `%USERPROFILE%\.wslconfig`:

```ini
[wsl2]
memory=8GB
swap=8GB
localhostForwarding=true

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
```

## Step 6: Verify Setup

Run the workflow manually:

1. Go to **Actions** → **Bootstrap End-to-End Test**
2. Click **Run workflow**
3. Set **run_windows_e2e** to `true`
4. (Optional) Set **windows_e2e_repo_fetch_ref** if you want to test a specific ref inside WSL
   - Examples:
     - `refs/heads/main`
     - `refs/tags/v1.2.3`
     - `refs/pull/123/merge`
5. Click **Run workflow**

The `bootstrap-windows-e2e` job should now run on your self-hosted runner.

## Runner Maintenance

### Cleaning Up Test Distributions

The workflow automatically cleans up test distributions, but if needed manually:

```powershell
# List all distributions
wsl --list --verbose

# Remove test distribution
wsl --unregister NixOS-Ripple-Test

# Clean up temp files
Remove-Item -Path "$env:TEMP\WSL-Test" -Recurse -Force
```

### Updating the Runner

```powershell
cd C:\actions-runner

# Stop the service
.\svc.cmd stop

# Remove the service
.\svc.cmd uninstall

# Download new version and reconfigure
# (Follow GitHub's update instructions)

# Reinstall service
.\svc.cmd install
.\svc.cmd start
```

### Troubleshooting

**Runner not picking up jobs:**
- Verify labels match: `[self-hosted, Windows, WSL2]`
- Check runner service is running: `.\svc.cmd status`
- Check runner logs in `_diag` folder

**WSL2 not working:**
- Run `wsl --status` to check WSL status
- Ensure Hyper-V and Virtual Machine Platform are enabled
- Try `wsl --update` to update WSL

**Disk space issues:**
- The test uses a 64GB sparse VHD (only uses actual data)
- Clean up old distributions: `wsl --unregister <name>`
- Check `%TEMP%\WSL-Test` for leftover files

**Permission issues:**
- Runner service needs Administrator privileges for WSL operations
- Configure service to run as Administrator or Local System

## Security Considerations

- Self-hosted runners execute code from your repository
- Only use on trusted repositories
- Consider network isolation for the runner machine
- Keep Windows and WSL updated
- The runner will have access to create/destroy WSL distributions

## Workflow Configuration

The E2E test job is configured in `.github/workflows/bootstrap-test.yml`:

```yaml
bootstrap-windows-e2e:
  name: Bootstrap Test (Windows WSL2 - E2E)
  runs-on: [self-hosted, Windows, WSL2]
  timeout-minutes: 90
  if: github.event_name == 'workflow_dispatch' && github.event.inputs.run_windows_e2e == 'true'
  needs: bootstrap-windows-syntax
```

The job:
1. Runs only when explicitly requested via `workflow_dispatch` (manual trigger)
2. Requires the syntax check to pass first
3. Uses labels `[self-hosted, Windows, WSL2]` to target the correct runner
4. Has a 90-minute timeout for the full installation process
5. Cleans up test distributions after completion

### Why the E2E job is manual-only

GitHub Actions cannot reliably "skip" a job that targets a self-hosted runner when no runner is online.
If this job is enabled on `push`/`pull_request`, it can queue indefinitely. Keeping it manual-only
prevents hanging CI when the Windows runner is offline.
