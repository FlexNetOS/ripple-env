# Rules and Guidelines

## Core Principles

### 1. Reproducibility First
- All configurations must be declarative
- Pin versions in lock files (`flake.lock`, `pixi.lock`)
- Avoid imperative state modifications
- Document any manual steps required

### 2. Cross-Platform Compatibility
- Test changes on Linux, macOS, and Windows/WSL2
- Use platform-agnostic patterns where possible
- Isolate platform-specific code in dedicated modules
- Never assume Unix-only environment

### 3. Modularity
- Keep modules focused and single-purpose
- Use clear interfaces between modules
- Enable/disable features via options
- Avoid tight coupling between components

### 4. Security
- Never commit secrets or credentials
- Use environment variables for sensitive data
- Validate all external inputs
- Keep dependencies updated

## Code Rules

### Nix Code

```
DO:
- Use lib functions for type safety
- Document all options
- Handle null/missing values gracefully
- Use mkIf for conditional configs

DON'T:
- Hardcode paths (use variables)
- Mix configuration with implementation
- Create circular dependencies
- Use deprecated syntax
```

### Shell Scripts

```
DO:
- Include shebang and set -euo pipefail
- Add --help documentation
- Quote all variables
- Use shellcheck for linting

DON'T:
- Use backticks (use $() instead)
- Rely on external state
- Ignore error codes
- Use eval with untrusted input
```

### PowerShell Scripts

```
DO:
- Include comment-based help
- Use approved verbs
- Support -WhatIf and -Confirm
- Handle errors with try/catch

DON'T:
- Use aliases in scripts
- Ignore $ErrorActionPreference
- Hardcode Windows paths
- Skip parameter validation
```

## Commit Rules

### Message Format
- Use conventional commits (feat, fix, docs, etc.)
- Keep subject line under 72 characters
- Use imperative mood ("add" not "added")
- Reference issues when applicable

### Content Rules
- One logical change per commit
- Don't mix formatting with functional changes
- Include tests for new features
- Update docs with code changes

## Pull Request Rules

### Before Opening
- [ ] Tests pass locally
- [ ] Linters pass
- [ ] Documentation updated
- [ ] Commit history is clean

### PR Description
- Explain the "why" not just the "what"
- Include test instructions
- Link related issues
- Add screenshots for UI changes

## File Organization

```
/
├── .claude/          # Agent configuration
├── .github/          # GitHub workflows and docs
├── lib/              # Nix library functions
├── modules/          # Home-manager modules
│   ├── common/       # Cross-platform
│   ├── linux/        # Linux-specific
│   └── macos/        # macOS-specific
├── bootstrap.sh      # Linux/macOS setup
├── bootstrap.ps1     # Windows setup
├── flake.nix         # Main flake
└── pixi.toml         # Pixi config
```

## Naming Conventions

### Files
- Nix files: `kebab-case.nix`
- Shell scripts: `kebab-case.sh`
- PowerShell: `PascalCase.ps1` or `kebab-case.ps1`

### Nix
- Options: `camelCase`
- Packages: `kebab-case`
- Functions: `camelCase`

### Variables
- Environment: `SCREAMING_SNAKE_CASE`
- Shell: `snake_case`
- PowerShell: `PascalCase`

## Dependency Rules

### Adding Dependencies
1. Check if already available in Nix
2. Prefer Nix packages over Pixi when possible
3. Pin specific versions for stability
4. Document why dependency is needed

### Updating Dependencies
1. Update one dependency at a time
2. Test thoroughly after updates
3. Update lock files atomically
4. Note breaking changes in commits

## Testing Requirements

### For New Features
- Unit tests where applicable
- Integration test in CI workflow
- Manual testing on target platforms
- Documentation of test procedures

### For Bug Fixes
- Add regression test
- Verify fix doesn't break other features
- Test on affected platforms

## Documentation Standards

### Code Comments
- Explain "why" not "what"
- Keep comments updated with code
- Use TODO/FIXME with tracking info
- Document non-obvious behavior

### README Updates
- Keep examples working
- Update after feature changes
- Include troubleshooting tips
- Link to detailed docs

## Performance Guidelines

### Nix Builds
- Use Nix cache effectively
- Minimize unnecessary rebuilds
- Profile slow builds
- Consider splitting large derivations

### Development Experience
- Keep dev shell startup fast
- Lazy-load where possible
- Use direnv for automatic activation
- Profile and optimize hot paths
