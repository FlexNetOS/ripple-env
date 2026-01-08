# Skills and Capabilities

## Environment Skills

### Nix Package Management
- Install/remove packages via `nix profile`
- Build derivations with `nix build`
- Enter development shells with `nix develop` / `nom develop`
- Check flake validity with `nix flake check`
- Update dependencies with `nix flake update`

### Pixi Package Management
- Add packages: `pixi add <package>`
- Remove packages: `pixi remove <package>`
- Search packages: `pixi search <pattern>`
- Update lockfile: `pixi update`
- Show environment: `pixi info`

### Shell Navigation
- Smart directory navigation with `zoxide` (`z <path>`)
- Fuzzy file finding with `fzf`
- File listing with `eza` (modern `ls`)
- File viewing with `bat` (modern `cat`)
- Ripgrep for fast searching (`rg <pattern>`)

## Development Skills

### ROS2 Development
- Build packages: `colcon build --symlink-install`
- Run tests: `colcon test`
- View results: `colcon test-result --verbose`
- Launch nodes: `ros2 launch <package> <launch_file>`
- Topic inspection: `ros2 topic list/echo/info`
- Service calls: `ros2 service call`
- Parameter management: `ros2 param`

### Git Operations
- Standard git workflow (add, commit, push, pull)
- GitHub CLI operations with `gh`
- PR creation: `gh pr create`
- Issue management: `gh issue`
- Workflow inspection: `gh run`

### Code Editing
- Helix editor with LSP support
- Language servers for: Python, C++, CMake, Nix, YAML, XML, Rust, Bash, TOML, Markdown
- Auto-completion, go-to-definition, hover docs
- Code formatting and linting

## Build Skills

### CMake Projects
- Configure: `cmake -B build -G Ninja`
- Build: `cmake --build build`
- Install: `cmake --install build`
- Test: `ctest --test-dir build`

### Python Projects
- Virtual environments via Pixi
- Package installation: `pixi add <package>`
- Running scripts: `pixi run python <script>`
- Testing: `pixi run pytest`

### C++ Projects
- Compilation with gcc/clang
- Debug builds with symbols
- Release optimization
- Static analysis with clang-tidy

## DevOps Skills

### CI/CD
- GitHub Actions workflow creation
- Workflow debugging and monitoring
- Secret management
- Artifact handling

### Container Operations
- Build images with Docker/Podman
- Multi-stage builds
- Layer optimization
- Registry push/pull

### Infrastructure
- Nix-based system configuration
- Home-manager user configuration
- Cross-platform deployments
- Reproducible environments

## Automation Skills

### Shell Scripting
- Bash script creation and debugging
- Nushell for structured data
- PowerShell for Windows automation
- Cross-platform script patterns

### Task Automation
- File watching and triggers
- Scheduled tasks
- Event-driven automation
- Workflow orchestration

## Analysis Skills

### Code Analysis
- Static analysis with various linters
- Dependency scanning
- Security vulnerability detection
- Performance profiling

### Log Analysis
- Pattern matching with ripgrep
- Structured log parsing
- Error aggregation
- Trend detection

## Communication Skills

### Documentation
- Markdown formatting
- Code documentation
- API documentation
- User guides and tutorials

### Reporting
- Status summaries
- Progress reports
- Error explanations
- Recommendations

## Tool Quick Reference

| Task | Command |
|------|---------|
| Enter dev shell | `nom develop` |
| Build ROS packages | `cb` or `colcon build --symlink-install` |
| Run tests | `ct` or `colcon test` |
| Test results | `ctr` or `colcon test-result --verbose` |
| Add ROS package | `pixi add ros-humble-<name>` |
| Search packages | `pixi search <pattern>` |
| Check flake | `nix flake check` |
| Update deps | `nix flake update` |
| Show ROS env | `ros2-env` |
| Git status | `git status` |
| Create PR | `gh pr create` |

## Skill Expansion

New skills can be added by:

1. **Nix packages**: Add to `flake.nix` or modules
2. **Pixi packages**: Add via `pixi add`
3. **Custom scripts**: Add to `scripts/` directory
4. **Shell functions**: Add to shell configuration modules
5. **Editor plugins**: Configure in editor module

## Limitations

Current limitations to be aware of:

- GUI applications require display forwarding in WSL2
- Some ROS packages may not be available in RoboStack
- Hardware access (USB, serial) needs additional configuration
- GPU acceleration requires driver setup
