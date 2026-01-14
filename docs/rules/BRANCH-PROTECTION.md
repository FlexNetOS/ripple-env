# Branch Protection Rules

This document describes the recommended branch protection rules for the `main` branch.

## GitHub Settings Configuration

Navigate to: **Settings > Branches > Add branch protection rule**

### Branch Name Pattern

```
main
```

### Protection Rules

#### Required Status Checks

Enable: **Require status checks to pass before merging**

Required checks:
- [ ] `flake-check` - Nix flake validation
- [ ] `ros2-build` - ROS2 build and test
- [ ] `security` - Trivy security scan
- [ ] `macos-build` - macOS compatibility (optional but recommended)

Enable: **Require branches to be up to date before merging**

#### Pull Request Reviews

Enable: **Require a pull request before merging**

- [x] Require approvals: **1**
- [ ] Dismiss stale pull request approvals when new commits are pushed
- [ ] Require review from Code Owners (if CODEOWNERS file exists)

#### Commit Signing

Enable: **Require signed commits** (recommended for security)

To set up commit signing:
```bash
# Generate GPG key
gpg --full-generate-key

# List keys
gpg --list-secret-keys --keyid-format=long

# Configure git
git config --global user.signingkey YOUR_KEY_ID
git config --global commit.gpgsign true

# Add public key to GitHub
gpg --armor --export YOUR_KEY_ID
# Copy output to GitHub > Settings > SSH and GPG keys > New GPG key
```

#### Additional Settings

- [ ] **Do not allow bypassing the above settings** (for strict enforcement)
- [x] **Restrict who can push to matching branches** (optional)
- [x] **Allow force pushes**: Disabled
- [x] **Allow deletions**: Disabled

## Recommended Workflow

### For Contributors

1. **Fork** the repository (external contributors)
2. **Create a feature branch**:
   ```bash
   git checkout -b feat/my-feature
   ```
3. **Make changes** and commit with conventional commits
4. **Push** to your fork/branch
5. **Create a Pull Request**
6. **Wait for CI** to pass
7. **Request review** from maintainers
8. **Address feedback** if needed
9. **Merge** once approved and CI passes

### For Maintainers

1. **Review** the PR changes
2. **Run additional tests** if needed
3. **Approve** or request changes
4. **Merge** using "Squash and merge" for clean history

## Status Check Details

### flake-check

Validates the Nix flake configuration:
- Syntax correctness
- All outputs build
- No deprecation warnings

### ros2-build

Builds and tests ROS2 packages:
- Installs Pixi environment
- Runs `colcon build`
- Runs `colcon test`
- Verifies pytest installation

### security

Runs security scans:
- Trivy filesystem scan
- CRITICAL and HIGH severity check
- Results uploaded to GitHub Security tab

### macos-build

Validates macOS compatibility:
- Builds devshell on macOS
- Verifies CUDA shell is unavailable (expected)
- Installs Pixi packages

## Enforcement

### What Happens on Violation

1. **Unsigned commits**: PR blocked until commits are signed
2. **Failing CI**: PR cannot be merged
3. **Stale branch**: Must rebase/merge with main before merging
4. **No reviews**: Must have at least 1 approval

### Exceptions

For emergency fixes:
1. Maintainer with admin access can bypass
2. Document the bypass in the PR
3. Follow up with proper review

## Security Considerations

1. **No force pushes**: Prevents history rewriting
2. **Signed commits**: Ensures commit authenticity
3. **Required reviews**: Prevents unreviewed code
4. **CI gates**: Prevents broken code in main

## Updating Rules

To modify branch protection:
1. Must have admin access
2. Navigate to Settings > Branches
3. Edit the `main` protection rule
4. Update this document to reflect changes

---

*Last updated: January 2026*
