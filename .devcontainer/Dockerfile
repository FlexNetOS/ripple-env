# NixOS-based Devcontainer for ripple-env
# Pure Nix environment - no apt, no Ubuntu, just Nix
#
# Build: docker build -t ripple-env-devcontainer .devcontainer/
# Run:   docker run -it ripple-env-devcontainer
#
FROM nixos/nix:2.33.0 AS base

# Configure Nix with flakes support
RUN mkdir -p /etc/nix && \
    echo "experimental-features = nix-command flakes" >> /etc/nix/nix.conf && \
    echo "accept-flake-config = true" >> /etc/nix/nix.conf && \
    echo "warn-dirty = false" >> /etc/nix/nix.conf && \
    echo "sandbox = false" >> /etc/nix/nix.conf

# Create non-root user (vscode for devcontainer compatibility)
#
# IMPORTANT (Codespaces): The nixos/nix image can have /etc/passwd and /etc/group
# implemented as symlinks. During Codespaces builds (with the Dev Containers
# CLI "Dockerfile-with-features" wrapper), these links can become cyclic,
# causing errors like:
#   groupadd: cannot open /etc/group: Too many levels of symbolic links
#
# To keep nixos/nix as our base while remaining Codespaces-friendly, we:
#   1) Ensure /etc/passwd and /etc/group are *regular files* (break symlink loops)
#   2) Add vscode user/group entries without relying on useradd/groupadd
RUN set -eu; \
        if [ -L /etc/passwd ] || [ -L /etc/group ]; then \
            rm -f /etc/passwd /etc/group; \
        fi; \
        if [ ! -f /etc/passwd ]; then \
            printf 'root:x:0:0:root:/root:/bin/sh\n' > /etc/passwd; \
        fi; \
        if [ ! -f /etc/group ]; then \
            printf 'root:x:0:\n' > /etc/group; \
        fi; \
        grep -q '^root:' /etc/passwd || printf 'root:x:0:0:root:/root:/bin/sh\n' >> /etc/passwd; \
        grep -q '^root:' /etc/group || printf 'root:x:0:\n' >> /etc/group; \
        grep -q '^vscode:' /etc/group || printf 'vscode:x:1000:\n' >> /etc/group; \
        grep -q '^vscode:' /etc/passwd || printf 'vscode:x:1000:1000::/home/vscode:/bin/sh\n' >> /etc/passwd; \
        mkdir -p /home/vscode; \
        chown -R 1000:1000 /home/vscode; \
        mkdir -p /etc/sudoers.d; \
        printf 'vscode ALL=(ALL) NOPASSWD:ALL\n' > /etc/sudoers.d/vscode; \
        chmod 0440 /etc/sudoers.d/vscode

# Install core system packages via Nix
# These mirror what's in nix/images/wsl.nix for consistency
RUN nix profile install --impure \
    nixpkgs#coreutils \
    nixpkgs#bashInteractive \
    nixpkgs#sudo \
    nixpkgs#shadow \
    nixpkgs#gnugrep \
    nixpkgs#gnused \
    nixpkgs#gawk \
    nixpkgs#findutils \
    nixpkgs#which \
    nixpkgs#less \
    nixpkgs#procps \
    nixpkgs#util-linux \
    nixpkgs#gnutar \
    nixpkgs#gzip \
    nixpkgs#xz \
    nixpkgs#cacert \
    nixpkgs#openssl

# Install development tools (matching WSL and flake packages)
RUN nix profile install --impure \
    nixpkgs#git \
    nixpkgs#git-lfs \
    nixpkgs#gh \
    nixpkgs#curl \
    nixpkgs#wget \
    nixpkgs#jq \
    nixpkgs#yq \
    nixpkgs#ripgrep \
    nixpkgs#fd \
    nixpkgs#bat \
    nixpkgs#eza \
    nixpkgs#htop \
    nixpkgs#btop

# Install Nix development tools
RUN nix profile install --impure \
    nixpkgs#direnv \
    nixpkgs#nix-direnv \
    nixpkgs#nix-output-monitor \
    nixpkgs#nix-tree \
    nixpkgs#nixfmt-rfc-style \
    nixpkgs#nil \
    nixpkgs#cachix

# Install shells
RUN nix profile install --impure \
    nixpkgs#zsh \
    nixpkgs#nushell \
    nixpkgs#starship \
    nixpkgs#zoxide

# Install editors
RUN nix profile install --impure \
    nixpkgs#helix \
    nixpkgs#neovim

# Install Python
RUN nix profile install --impure \
    nixpkgs#python313 \
    nixpkgs#python313Packages.pip \
    nixpkgs#python313Packages.virtualenv

# Install build tools and compilation cache
RUN nix profile install --impure \
    nixpkgs#gcc13 \
    nixpkgs#gnumake \
    nixpkgs#cmake \
    nixpkgs#ninja \
    nixpkgs#pkg-config \
    nixpkgs#ccache \
    nixpkgs#sccache \
    nixpkgs#mold

# Install Kubernetes tools
RUN nix profile install --impure \
    nixpkgs#kubectl \
    nixpkgs#kubernetes-helm \
    nixpkgs#kustomize \
    nixpkgs#k9s \
    nixpkgs#kubectx

# Install version control extras
RUN nix profile install --impure \
    nixpkgs#jujutsu \
    nixpkgs#lazygit

# Install Rust toolchain (from dev-tools.nix)
RUN nix profile install --impure \
    nixpkgs#cargo \
    nixpkgs#rustc \
    nixpkgs#rust-analyzer \
    nixpkgs#rustfmt \
    nixpkgs#clippy \
    nixpkgs#maturin \
    nixpkgs#sqlx-cli

# Install Node.js ecosystem (from dev-tools.nix)
RUN nix profile install --impure \
    nixpkgs#nodejs_22 \
    nixpkgs#nodePackages.pnpm

# Install distributed systems tools (from distributed.nix)
RUN nix profile install --impure \
    nixpkgs#grpcurl \
    nixpkgs#grpcui \
    nixpkgs#protobuf \
    nixpkgs#protoc-gen-go \
    nixpkgs#protoc-gen-go-grpc \
    nixpkgs#wasm-pack \
    nixpkgs#wasm-bindgen-cli \
    nixpkgs#binaryen \
    nixpkgs#websocat \
    nixpkgs#graphviz

# Install cryptography and security tools (from distributed.nix)
RUN nix profile install --impure \
    nixpkgs#libsodium \
    nixpkgs#age \
    nixpkgs#sops \
    nixpkgs#vault \
    nixpkgs#step-cli \
    nixpkgs#cosign \
    nixpkgs#trivy \
    nixpkgs#syft \
    nixpkgs#grype \
    nixpkgs#opa

# Install hardware interface tools (from hardware.nix)
RUN nix profile install --impure \
    nixpkgs#libv4l \
    nixpkgs#v4l-utils \
    nixpkgs#gphoto2 \
    nixpkgs#can-utils \
    nixpkgs#socat \
    nixpkgs#screen \
    nixpkgs#minicom \
    nixpkgs#picocom \
    nixpkgs#usbutils \
    nixpkgs#libusb1 \
    nixpkgs#libgpiod \
    nixpkgs#i2c-tools

# Install simulation tools (from simulation.nix)
RUN nix profile install --impure \
    nixpkgs#ogre \
    nixpkgs#assimp \
    nixpkgs#mesa \
    nixpkgs#glxinfo \
    nixpkgs#xorg.xeyes \
    nixpkgs#xdotool \
    nixpkgs#bullet \
    nixpkgs#ode \
    nixpkgs#ffmpeg \
    nixpkgs#imagemagick \
    nixpkgs#xvfb-run

# Install infrastructure and monitoring (from base.nix and dev-tools.nix)
RUN nix profile install --impure \
    nixpkgs#prometheus \
    nixpkgs#natscli \
    nixpkgs#nats-server \
    nixpkgs#trippy \
    nixpkgs#kubo \
    nixpkgs#sqlite \
    nixpkgs#tree-sitter

# Install additional shell and dev utilities (from dev-tools.nix)
RUN nix profile install --impure \
    nixpkgs#fzf \
    nixpkgs#unzip \
    nixpkgs#portaudio \
    nixpkgs#devpod \
    nixpkgs#runc

# Install configuration management (from base.nix)
RUN nix profile install --impure \
    nixpkgs#home-manager \
    nixpkgs#chezmoi \
    nixpkgs#stow \
    nixpkgs#shellcheck \
    nixpkgs#yamllint

# Install AI/ML tools (from dev-tools.nix)
# Note: local-ai is a large package (~500MB+) and may not be available on all platforms
RUN nix profile install --impure \
    nixpkgs#aichat
RUN ( nix profile install --impure nixpkgs#aider-chat || echo "Warning: failed to install nixpkgs#aider-chat; continuing without aider-chat." )
RUN nix profile install --impure nixpkgs#local-ai || true

# Install Linux platform-specific tools (from platform.nix)
RUN nix profile install --impure \
    nixpkgs#inotify-tools \
    nixpkgs#strace \
    nixpkgs#gdb \
    nixpkgs#gvisor

# Note: Holochain packages (holochain, hc, lair-keystore) require the Holochain overlay
# These are available in the flake's development shells via `nix develop`
# To use them in the devcontainer, run: `nix develop` to enter the shell
# Or install manually: nix profile install github:spartan-holochain-counsel/nix-overlay#holochain

# Install Pixi package manager
RUN curl -fsSL https://pixi.sh/install.sh | bash && \
    mv /root/.pixi/bin/pixi /nix/var/nix/profiles/default/bin/ || true

# Set up user environment
USER vscode
WORKDIR /home/vscode

# Create Nix profile symlink for user
RUN mkdir -p ~/.nix-profile && \
    ln -sf /nix/var/nix/profiles/default ~/.nix-profile/default || true

# Configure shell integration
RUN mkdir -p ~/.config/nushell ~/.config/direnv

# Zsh configuration with direnv hook
RUN echo 'export PATH="/nix/var/nix/profiles/default/bin:$HOME/.pixi/bin:$HOME/.local/bin:$PATH"' >> ~/.zshrc && \
    echo 'eval "$(direnv hook zsh)"' >> ~/.zshrc && \
    echo 'eval "$(starship init zsh)"' >> ~/.zshrc && \
    echo 'eval "$(zoxide init zsh)"' >> ~/.zshrc && \
    echo 'export EDITOR=hx VISUAL=hx' >> ~/.zshrc

# Bash configuration with direnv hook
RUN echo 'export PATH="/nix/var/nix/profiles/default/bin:$HOME/.pixi/bin:$HOME/.local/bin:$PATH"' >> ~/.bashrc && \
    echo 'eval "$(direnv hook bash)"' >> ~/.bashrc && \
    echo 'eval "$(starship init bash)"' >> ~/.bashrc && \
    echo 'eval "$(zoxide init bash)"' >> ~/.bashrc && \
    echo 'export EDITOR=hx VISUAL=hx' >> ~/.bashrc

# Nushell env configuration
RUN echo '$env.PATH = ($env.PATH | prepend "/nix/var/nix/profiles/default/bin" | prepend $"($env.HOME)/.pixi/bin" | prepend $"($env.HOME)/.local/bin")' > ~/.config/nushell/env.nu && \
    echo '$env.EDITOR = "hx"' >> ~/.config/nushell/env.nu && \
    echo '$env.VISUAL = "hx"' >> ~/.config/nushell/env.nu

# Nushell config
RUN echo '$env.config = { show_banner: false }' > ~/.config/nushell/config.nu && \
    echo 'alias ll = ls -l' >> ~/.config/nushell/config.nu && \
    echo 'alias la = ls -la' >> ~/.config/nushell/config.nu

# Direnv configuration
RUN echo '[whitelist]' > ~/.config/direnv/direnv.toml && \
    echo 'prefix = ["/workspaces"]' >> ~/.config/direnv/direnv.toml

# Git LFS setup
RUN git lfs install --skip-repo

# Set default shell to zsh
ENV SHELL=/nix/var/nix/profiles/default/bin/zsh
ENV PATH="/nix/var/nix/profiles/default/bin:/home/vscode/.pixi/bin:/home/vscode/.local/bin:${PATH}"
ENV NIX_PATH="nixpkgs=channel:nixpkgs-unstable"
ENV EDITOR="hx"
ENV VISUAL="hx"

# SSL certificates for curl/git
ENV SSL_CERT_FILE="/nix/var/nix/profiles/default/etc/ssl/certs/ca-bundle.crt"
ENV NIX_SSL_CERT_FILE="/nix/var/nix/profiles/default/etc/ssl/certs/ca-bundle.crt"

WORKDIR /workspaces

# Basic healthcheck so container runtimes can detect broken images.
# (Trivy recommends having an explicit HEALTHCHECK.)
HEALTHCHECK --interval=30s --timeout=5s --start-period=30s --retries=3 \
    CMD ["/nix/var/nix/profiles/default/bin/bash", "-lc", "nix --version >/dev/null"]

# Default command
CMD ["/nix/var/nix/profiles/default/bin/zsh"]
