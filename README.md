# ros2-humble-env

a reproducible and declarative development environment for ros2 humble using nix flakes and pixi for cross-platform compatibility. this repository is meant to be used as a "template" repository for robotics projects to offer a easy starting environment with a working ros2 install.

## overview

this repository provides a complete development setup for ros2 humble with all necessary build tools and dependencies pre-configured. the environment works on linux, macos (x86 and arm64), and wsl2 using:

- **nix flakes**: for reproducible, declarative environment setup
- **pixi**: for python and ros package management via robostack
- **neovim + lazyvim**: for ide functionality with ros2, python, c/c++, and web ui support
- **direnv** (recommended): for automatic environment activation

## quick start

### bootstrap (recommended)

the easiest way to get started is using the bootstrap scripts. they will verify prerequisites, set up the environment, and recommend using direnv for automatic activation:

**linux/macos/wsl2:**
```bash
./bootstrap.sh
```

**windows (powershell/wsl2):**
```powershell
./bootstrap.ps1
```

these scripts will:
1. verify nix is installed with flakes enabled
2. check for direnv (recommended) and provide installation instructions if missing
3. enter the nix dev shell
4. run pixi install to set up ros2 and dependencies
5. provide clear error messages if prerequisites are missing

after bootstrap completes:
- **if you have direnv**: run `direnv allow` and the environment will activate automatically whenever you cd into the directory
- **without direnv**: use `nix develop` to manually enter the environment

### manual setup

#### prerequisites

- nix with flakes enabled
- git
- direnv (recommended for automatic environment activation)

#### installing direnv (recommended)

direnv automatically loads the development environment when you enter the directory:

**macos:**
```bash
brew install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc  # or ~/.zshrc for zsh
```

**ubuntu/debian:**
```bash
sudo apt install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc  # or ~/.zshrc for zsh
```

**fedora:**
```bash
sudo dnf install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc  # or ~/.zshrc for zsh
```

**arch linux:**
```bash
sudo pacman -S direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc  # or ~/.zshrc for zsh
```

after installing direnv, restart your shell or source your shell config file.

#### installing nix (if not already installed)

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

## neovim ide setup

the development environment includes neovim with lazyvim for a complete ide experience. to set up lazyvim:

1. enter the dev shell:
   ```bash
   nix develop
   ```

2. run the setup command:
   ```bash
   setup-lazyvim
   ```

3. launch neovim:
   ```bash
   nvim
   ```

lazyvim will automatically install plugins on first launch, including:
- **lsp**: language server protocol support for ros2, python, c/c++
- **treesitter**: advanced syntax highlighting
- **telescope**: fuzzy finder for files and content
- **git integration**: fugitive and gitsigns
- **terminal**: integrated terminal support
- **preview**: markdown and other file previews

the configuration is stored in `~/.config/nvim` and can be customized to your needs.

## shell policy

**default interactive shell**: the development environment defaults to **bash/zsh** for interactive use. you can use your preferred interactive shell (bash, zsh, fish) as usual.

**nushell for automation**: nushell is included specifically for automation and workflow scripts. it should be used non-interactively for scripting purposes.

### using nushell for automation

nushell is available in the dev shell for automation tasks:

```bash
# example: non-interactive workflow command
nu -c "ls | where type == file | get name"

# example: batch processing ros2 packages
nu -c "ls src | each { |pkg| echo $'Building ($pkg.name)' }"

# example: environment inspection
nu -c "env | where name =~ ROS"
```

### using other shells interactively

if you want to use a different interactive shell (like zsh, fish, or nushell), you have a few options:

### manual approach

```bash
nix develop -c env 'SHELL=<your-shell-path-here>' <your-shell-path-here>
```

### create an alias

add this alias to your shell configuration for easier access:

```bash
alias devshell="nix develop -c env 'SHELL=/bin/bash' /bin/bash"
```

## bash function

you can also create a bash function for easier access. add this to your `.bashrc` or `.bash_profile`:

```bash
devshell() {
  local shell_path="/bin/bash"  # change this to your preferred shell path
  nix develop -c env "SHELL=$shell_path" "$shell_path"
}
```

this should do the same as the alias defined above but as a function.

### with nom

if you use nom for better nix output, replace `nix` with `nom`:

```bash
nom develop -c env 'SHELL=/bin/bash' /bin/bash
```

## adding packages

the environment uses robostack to provide ros2 humble packages. to add a package:

```bash
pixi add <PACKAGE_NAME>
```

find available ros2-humble packages in the [robostack channel](https://robostack.github.io/humble.html).

## environment details

the workspace includes:

- **ros**: ros-humble-desktop with all core ros packages
- **build tools**: cmake, ninja, make, compilers, pkg-config
- **ros tools**: colcon, rosdep, catkin_tools
- **python**: 3.11.x with development headers
- **ide**: neovim with lazyvim, lsp, treesitter, telescope
- **automation**: nushell for workflow scripts
- **platforms**: supports linux-64, linux-aarch64, osx-64, osx-arm64

## workspace structure

```
ros2-humble-env/
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

## links

- [robostack ros2-humble packages](https://robostack.github.io/humble.html)
- [pixi documentation](https://pixi.sh)
- [nix flakes documentation](https://nixos.wiki/wiki/Flakes)
- [ros2 humble documentation](https://docs.ros.org/en/humble/)
- [flake-parts devshell documentation](https://flake.parts/options/devshell.html)

## license

this project is licensed under the [MIT License](LICENSE). see the LICENSE file for details.
