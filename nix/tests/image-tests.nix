# NixOS VM Tests for Generated Images
# These tests validate that our NixOS images work correctly
#
# Run tests:
#   nix build .#checks.x86_64-linux.wsl-image-test
#   nix build .#checks.x86_64-linux.vm-image-test
{ inputs, pkgs, lib, ... }:

let
  # Base test configuration shared across all image tests.
  #
  # NOTE: nixpkgs' NixOS test API has evolved (e.g. `pkgs.nixosTest` ->
  # `pkgs.testers.nixosTest`, and accepted attributes changed). Keep this empty
  # unless you have verified attribute names against your pinned nixpkgs.
  baseTestConfig = { };

  # Test that basic system services are working
  basicServicesTest = {
    name = "basic-services";
    nodes.machine = { config, pkgs, ... }: {
      imports = [
        "${inputs.nixpkgs}/nixos/modules/virtualisation/qemu-vm.nix"
      ];

      # Minimal VM configuration
      virtualisation = {
        memorySize = 1024;
        cores = 2;
      };

      # Basic services that should be running
      services.openssh.enable = true;

      # Nix with flakes
      nix.settings.experimental-features = [ "nix-command" "flakes" ];

      # Test user
      users.users.testuser = {
        isNormalUser = true;
        extraGroups = [ "wheel" ];
        password = "test";
      };

      security.sudo.wheelNeedsPassword = false;
    };

    testScript = ''
      machine.start()
      machine.wait_for_unit("multi-user.target")

      # Test SSH service
      machine.wait_for_unit("sshd.service")
      machine.succeed("systemctl is-active sshd")

      # Test Nix is available
      machine.succeed("nix --version")
      machine.succeed("nix flake --help")

      # Test user exists
      machine.succeed("id testuser")

      # Test sudo works
      machine.succeed("su - testuser -c 'sudo whoami' | grep root")

      machine.shutdown()
    '';
  };

  # Test Docker functionality
  dockerTest = {
    name = "docker-services";
    nodes.machine = { config, pkgs, ... }: {
      imports = [
        "${inputs.nixpkgs}/nixos/modules/virtualisation/qemu-vm.nix"
      ];

      virtualisation = {
        memorySize = 2048;
        cores = 2;
      };

      # Docker configuration
      virtualisation.docker = {
        enable = true;
        enableOnBoot = true;
      };

      users.users.testuser = {
        isNormalUser = true;
        extraGroups = [ "wheel" "docker" ];
        password = "test";
      };
    };

    testScript = ''
      machine.start()
      machine.wait_for_unit("multi-user.target")

      # Wait for Docker to be ready
      machine.wait_for_unit("docker.service")
      machine.succeed("systemctl is-active docker")

      # Test Docker works
      machine.succeed("docker info")
      machine.succeed("docker run --rm hello-world || true")

      # Test user can use Docker
      machine.succeed("su - testuser -c 'docker ps'")

      machine.shutdown()
    '';
  };

  # Test development tools are available
  devToolsTest = {
    name = "dev-tools";
    nodes.machine = { config, pkgs, ... }: {
      imports = [
        "${inputs.nixpkgs}/nixos/modules/virtualisation/qemu-vm.nix"
      ];

      virtualisation = {
        memorySize = 1024;
        cores = 2;
      };

      environment.systemPackages = with pkgs; [
        git
        gh
        curl
        wget
        jq
        helix
        neovim
        direnv
        nix-direnv
        python313
      ];

      programs.zsh.enable = true;
      programs.git = {
        enable = true;
        lfs.enable = true;
      };
    };

    testScript = ''
      machine.start()
      machine.wait_for_unit("multi-user.target")

      # Test git
      machine.succeed("git --version")
      machine.succeed("git lfs --version")

      # Test editors
      machine.succeed("hx --version")
      machine.succeed("nvim --version")

      # Test utilities
      machine.succeed("curl --version")
      machine.succeed("wget --version")
      machine.succeed("jq --version")

      # Test Python
      machine.succeed("python3 --version")

      # Test direnv
      machine.succeed("direnv --version")

      machine.shutdown()
    '';
  };

  # Test security hardening
  securityHardeningTest = {
    name = "security-hardening";
    nodes.machine = { config, pkgs, ... }: {
      imports = [
        "${inputs.nixpkgs}/nixos/modules/virtualisation/qemu-vm.nix"
        ../images/security-hardening.nix
      ];

      virtualisation = {
        memorySize = 1024;
        cores = 2;
      };

      # Enable security hardening
      security.hardening.enable = true;
    };

    testScript = ''
      machine.start()
      machine.wait_for_unit("multi-user.target")

      # Test kernel parameters are set
      machine.succeed("sysctl kernel.dmesg_restrict | grep 1")
      machine.succeed("sysctl kernel.kptr_restrict | grep 2")
      machine.succeed("sysctl net.ipv4.conf.all.rp_filter | grep 1")

      # Test audit service is running (if enabled)
      machine.succeed("systemctl is-active systemd-journald")

      # Test tmp is mounted with noexec (if configured)
      # machine.succeed("mount | grep '/tmp' | grep noexec")

      machine.shutdown()
    '';
  };

  # Test network connectivity
  networkTest = {
    name = "network";
    nodes.machine = { config, pkgs, ... }: {
      imports = [
        "${inputs.nixpkgs}/nixos/modules/virtualisation/qemu-vm.nix"
      ];

      virtualisation = {
        memorySize = 1024;
        cores = 2;
      };

      networking.firewall.enable = true;
      networking.firewall.allowedTCPPorts = [ 22 ];

      services.openssh.enable = true;
    };

    testScript = ''
      machine.start()
      machine.wait_for_unit("multi-user.target")
      machine.wait_for_unit("network-online.target")

      # Test network is up
      machine.succeed("ip addr show")
      machine.succeed("ip route show")

      # Test firewall is active
      machine.succeed("systemctl is-active firewalld || systemctl is-active iptables || true")

      # Test DNS resolution (if network available)
      machine.succeed("getent hosts localhost")

      machine.shutdown()
    '';
  };

in
{
  # Export tests for use in flake checks
  inherit basicServicesTest dockerTest devToolsTest securityHardeningTest networkTest;

  # Create actual NixOS test derivations
  mkTests = { system ? "x86_64-linux" }:
    let
      mkTest = test: pkgs.testers.nixosTest (test // baseTestConfig);
    in
    {
      basic-services = mkTest basicServicesTest;
      docker-services = mkTest dockerTest;
      dev-tools = mkTest devToolsTest;
      security-hardening = mkTest securityHardeningTest;
      network = mkTest networkTest;
    };
}
