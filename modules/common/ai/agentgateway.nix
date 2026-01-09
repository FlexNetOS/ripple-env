{ pkgs, lib, config, ... }:
let
  inherit (lib) mkEnableOption mkIf;
  cfg = config.programs.agentgateway;

  # Define the package derivation locally since it's not in nixpkgs yet
  agentgateway = pkgs.rustPlatform.buildRustPackage rec {
    pname = "agentgateway";
    version = "0.1.0-unstable-2025-01-09";

    src = pkgs.fetchFromGitHub {
      owner = "agentgateway";
      repo = "agentgateway";
      rev = "bb8fd2d45c5643f48075f17738d6c4d64be23c30"; # HEAD as of 2025-01-09
      # Use lib.fakeHash to trigger hash calculation on first build
      # After build fails, replace with the "got:" hash from the error message
      hash = lib.fakeHash;
    };

    # Use lib.fakeHash to trigger hash calculation on first build
    # After build fails, replace with the "got:" hash from the error message
    cargoHash = lib.fakeHash;

    nativeBuildInputs = with pkgs; [
      pkg-config
      protobuf
    ];

    buildInputs = with pkgs; [
      openssl
    ];

    # Run tests, but try to avoid network-dependent ones.
    # If the upstream project uses tests named or grouped with "network"/"online",
    # they will be skipped; otherwise, all tests run as normal.
    doCheck = true;
    checkPhase = ''
      cargo test --offline --tests -- --skip network --skip online
    '';

    meta = with lib; {
      description = "Model Context Protocol (MCP) Gateway";
      homepage = "https://github.com/agentgateway/agentgateway";
      license = licenses.mit; # Assuming MIT, check repo
      maintainers = with maintainers; [ ];
      mainProgram = "agentgateway";
    };
  };
in
{
  options.programs.agentgateway = {
    enable = mkEnableOption "Agent Gateway (MCP Server)";
  };

  config = mkIf cfg.enable {
    home.packages = [ agentgateway ];

    # Example configuration file
    xdg.configFile."agentgateway/config.yaml".text = ''
      # Agent Gateway Configuration
      # See https://github.com/agentgateway/agentgateway for details

      server:
        port: 8080
        host: "127.0.0.1"

      # Add tool configurations here
      tools: []
    '';
  };
}
