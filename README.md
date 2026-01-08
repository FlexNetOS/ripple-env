# Riple Enviroment and Bianry Creation

- Uses the orginal environment template setup from "ros2-humble-env" to create a wsl2 distro. 
- a reproducible and declarative development environment for ros2 humble using nix flakes and pixi for cross-platform compatibility. 
- this repository is meant to be used as a "template" repository for robotics projects to offer a easy starting environment with a working ros2 install.

# Goal: 

- A single script run from windows powershell that checks for linux and wsl2 install, then installs and updates them as needed.
- Creates the NixOS Distro, reigisters it, Creates the ext4.vhxd hard disk image 1TB, and swap image. 
- Loads the nix flake, and configurqations per direnv
- sets up pixi package manager, tools, and packages
- adds zsh and nushell (bash stays default with nix)
- Uses nom instead of nix
- installs git and gh cli
- Add all configs/files from "https://github.com/GustavoWidman/nix" then search "https://github.com/RGBCube/ncc" and add what is still missing. 

## overview

this repository provides a complete development setup for ros2 humble with all necessary build tools and dependencies pre-configured. the environment seems to work properly both on linux and macos (x86 and arm64) using:

- **nix flakes**: for reproducible, declarative environment setup
- **pixi**: for python and ros package management via robostack
- **direnv**: for automatic environment activation

## getting started

### check list

- nix with flakes enabled
- Git >= 2.19.0
- nom
- pixi
- nushell
- zsh
- bash (included by default nix)

### installing nix

personally, i like to use the [experimental nix install script](https://github.com/NixOS/experimental-nix-installer):

```bash
curl --proto '=https' --tlsv1.2 -sSf -L https://artifacts.nixos.org/experimental-installer | \
  sh -s -- install
```

it installs nix with `flakes` and `nix-command` enabled by default. it also offers an easy-to-use uninstaller in case you decide you don't want it anymore (`/nix/nix-installer uninstall`). alternatively, you can follow the [official nix installation instructions](https://nixos.org/download.html).

### enter the development environment

#### using direnv

if you have direnv installed, simply enter the directory:

```bash
cd ros2-humble-env
```

direnv will automatically load the environment. on first run, you may need to:

```bash
direnv allow
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
nom develop -c env 'SHELL=<your-shell-path-here>' <your-shell-path-here>
```

for example, with nushell:

```bash
nom develop -c env 'SHELL=/bin/nu' /bin/nu
```

### create an alias

add this alias to your shell configuration for easier access:

```bash
alias devshell="nom develop -c env 'SHELL=/bin/bash' /bin/bash"
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


## environment details

the workspace includes:

- **ros**: ros-humble-desktop with all core ros packages
- **build tools**: cmake, ninja, make, compilers, pkg-config
- **ros tools**: colcon, rosdep, catkin_tools
- **python**: 3.11.x with development headers
- **platforms**: supports linux-64, linux-aarch64, osx-64, osx-arm64

## workspace structure **Expand after adding configs/files from "https://github.com/GustavoWidman/nix" & "https://github.com/RGBCube/ncc"

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

## Add all missing packages to pixi's env. e.g., python, cargo, and so one. 
Example: let's say you need `pygame`, for example, then you'd add it by doing `pixi add pygame` (which would alter the `pixi.toml` and `pixi.lock` files).

## links

- [robostack ros2-humble packages](https://robostack.github.io/humble.html)
- [pixi documentation](https://pixi.sh)
- [nix flakes documentation](https://nixos.wiki/wiki/Flakes)
- [ros2 humble documentation](https://docs.ros.org/en/humble/)
- [flake-parts devshell documentation](https://flake.parts/options/devshell.html)

## license

this project is licensed under the [MIT License](LICENSE). see the LICENSE file for details.
