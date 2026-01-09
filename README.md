# ros2-humble-env

a reproducible and declarative development environment for ros2 humble using nix flakes and pixi for cross-platform compatibility. this repository is meant to be used as a "template" repository for robotics projects to offer a easy starting environment with a working ros2 install.

## overview

this repository provides a complete development setup for ros2 humble with all necessary build tools and dependencies pre-configured. the environment seems to work properly both on linux and macos (x86 and arm64) using:

- **nix flakes**: for reproducible, declarative environment setup
- **pixi**: for python and ros package management via robostack
- **direnv** (optional): for automatic environment activation

## getting started

### prerequisites

- nix with flakes enabled
- git

### installing nix (if not already installed)

personally, i like to use the [experimental nix install script](https://github.com/NixOS/experimental-nix-installer):

```bash
curl --proto '=https' --tlsv1.2 -sSf -L https://artifacts.nixos.org/experimental-installer | \
  sh -s -- install
```

it installs nix with `flakes` and `nix-command` enabled by default. it also offers an easy-to-use uninstaller in case you decide you don't want it anymore (`/nix/nix-installer uninstall`). alternatively, you can follow the [official nix installation instructions](https://nixos.org/download.html).

### enter the development environment

#### using direnv (recommended)

if you have direnv installed, simply enter the directory:

```bash
cd ros2-humble-env
```

direnv will automatically load the environment. on first run, you may need to:

```bash
direnv allow
```

#### using nix develop

alternatively, activate the environment manually:

```bash
nix develop
```

or with `nom` for better build output:

```bash
nom develop
```

## adding packages

the environment uses robostack to provide ros2 humble packages. to add a package:

```bash
pixi add <PACKAGE_NAME>
```

find available ros2-humble packages in the [robostack channel](https://robostack.github.io/humble.html).

## using a different shell

nix shells default to bash, but you can use your preferred shell. if you want to use a different shell (like zsh, fish, or nushell), you have a few options:

### manual approach

```bash
nix develop -c env 'SHELL=<your-shell-path-here>' <your-shell-path-here>
```

for example, with nushell:

```bash
nix develop -c env 'SHELL=/bin/nu' /bin/nu
```

### create an alias

add this alias to your shell configuration for easier access:

```bash
alias devshell="nix develop -c env 'SHELL=/bin/bash' /bin/bash"
```

### nushell function

if you use nushell, create this function in your `env.nu`:

```nu
def devshell [] {
  let shell_path = "/bin/nu"
  nix develop -c env $'SHELL=($shell_path)' $shell_path
}
```

then simply run `devshell` to enter the environment with nushell.

## bash function

you can also create a bash function for easier access. add this to your `.bashrc` or `.bash_profile`:

```bash
devshell() {
  local shell_path="/bin/bash"  # change this to your preferred shell path
  nix develop -c env "SHELL=$shell_path" "$shell_path"
}
```

this should do the same as the nushell function defined above but for bash users.

### with nom

if you use nom for better nix output, replace `nix` with `nom`:

```bash
nom develop -c env 'SHELL=/bin/nu' /bin/nu
```

## environment details

the workspace includes:

- **ros**: ros-humble-desktop with all core ros packages
- **build tools**: cmake, ninja, make, compilers, pkg-config
- **ros tools**: colcon, rosdep, catkin_tools
- **python**: 3.11.x with development headers
- **python modernization tools**: ruff, pyupgrade, flynt (for upgrading legacy python code)
- **platforms**: supports linux-64, linux-aarch64, osx-64, osx-arm64

## workspace structure

```
ros2-humble-env/
├── .claude/
│   └── skills/        # claude code skills for python modernization
│       ├── python-ruff-tool/
│       ├── python-pyupgrade-tool/
│       └── python-flynt-tool/
├── flake.nix          # nix flake configuration
├── pixi.toml          # pixi workspace definition
├── pixi.lock          # locked dependency versions
└── README.md          # this file
```

## useful commands

once in the development environment, sourcing ros is not needed as that is all handled by pixi automatically.

```bash
# build ros packages with colcon
colcon build

# list all available ros humble robostack packages
pixi search ros-humble-*

# search for a specific ros2 package, like all packages containing "rosidl" in the name
pixi search *rosidl*

# add a new package
pixi add ros-humble-<package-name>
```

it's worth noting that it's also recommended to add python packages to pixi's env as well. let's say you need `pygame`, for example, then you'd add it by doing `pixi add pygame` (which would alter the `pixi.toml` and `pixi.lock` files).

## python modernization tools

this environment includes tools for upgrading legacy python code to modern syntax. these tools are particularly useful when working with ros packages that may use older python patterns.

### ruff

an extremely fast python linter and formatter (10-100x faster than flake8/black), written in rust. it includes pyupgrade rules built-in.

```bash
# lint python files
ruff check .

# auto-fix issues
ruff check --fix .

# format code (like black)
ruff format .

# upgrade python syntax (using UP rules)
ruff check --select UP --fix .
```

### pyupgrade

automatically upgrades python syntax to newer versions of the language.

```bash
# upgrade to python 3.11+ syntax
find . -name "*.py" -exec pyupgrade --py311-plus {} +

# examples of transformations:
# - set([1, 2]) -> {1, 2}
# - class Foo(object) -> class Foo
# - super(Foo, self) -> super()
# - Optional[int] -> int | None (3.10+)
# - "{}".format(x) -> f"{x}"
```

### flynt

converts old-style string formatting (% and .format()) to f-strings.

```bash
# convert a file
flynt file.py

# convert entire directory
flynt src/

# dry run (see changes without modifying)
flynt --dry-run src/

# also convert string concatenation
flynt --transform-concats src/
```

### combined workflow

for a complete modernization of a python codebase:

```bash
# step 1: run ruff to fix common issues
ruff check --fix .

# step 2: upgrade syntax with pyupgrade
find . -name "*.py" -not -path "./.venv/*" -exec pyupgrade --py311-plus {} +

# step 3: convert to f-strings with flynt
flynt .

# step 4: format with ruff
ruff format .

# step 5: final lint check
ruff check .
```

## links

- [robostack ros2-humble packages](https://robostack.github.io/humble.html)
- [pixi documentation](https://pixi.sh)
- [nix flakes documentation](https://nixos.wiki/wiki/Flakes)
- [ros2 humble documentation](https://docs.ros.org/en/humble/)
- [flake-parts devshell documentation](https://flake.parts/options/devshell.html)
- [ruff documentation](https://docs.astral.sh/ruff/)
- [pyupgrade repository](https://github.com/asottile/pyupgrade)
- [flynt repository](https://github.com/ikamensh/flynt)

## license

this project is licensed under the [MIT License](LICENSE). see the LICENSE file for details.
