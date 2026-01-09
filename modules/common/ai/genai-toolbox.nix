{ pkgs, lib, config, ... }:
let
  inherit (lib) mkEnableOption mkIf;
  cfg = config.programs.genai-toolbox;

  # Define the package derivation locally since it's not in nixpkgs yet
  genai-toolbox = pkgs.buildGoModule rec {
    pname = "genai-toolbox";
    version = "0.25.0";

    src = pkgs.fetchFromGitHub {
      owner = "googleapis";
      repo = "genai-toolbox";
      rev = "v${version}";
      hash = "sha256-AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="; # Placeholder - will fail with correct hash
    };

    # Use lib.fakeHash to trigger hash calculation on first build
    # After build fails, replace with the "got:" hash from the error message
    vendorHash = lib.fakeHash;

    # The main binary is named 'server' in the repository
    # We'll keep it as 'genai-toolbox' or 'mcp-toolbox' for clarity
    postInstall = ''
      # Rename binary if needed for better CLI experience
      if [ -f $out/bin/server ]; then
        mv $out/bin/server $out/bin/mcp-toolbox
      fi
    '';

    nativeBuildInputs = with pkgs; [
      pkg-config
    ];

    buildInputs = with pkgs; [
      # No specific build dependencies needed for pure Go build
    ];

    # Skip tests that require external services (databases, cloud APIs)
    doCheck = false;

    ldflags = [
      "-s"
      "-w"
      "-X main.version=${version}"
    ];

    meta = with lib; {
      description = "MCP Toolbox for Databases - AI tool integration for database access";
      longDescription = ''
        MCP Toolbox for Databases is an open-source server that simplifies building
        AI tools with database access. It handles connection pooling, authentication,
        and provides OpenTelemetry observability support.

        Part of the Model Context Protocol (MCP) ecosystem for Layer 8 tool execution.
      '';
      homepage = "https://github.com/googleapis/genai-toolbox";
      license = licenses.asl20; # Apache 2.0
      maintainers = with maintainers; [ ];
      mainProgram = "mcp-toolbox";
      platforms = platforms.unix;
    };
  };
in
{
  options.programs.genai-toolbox = {
    enable = mkEnableOption "MCP Toolbox (genai-toolbox) for Layer 8 tool execution";
  };

  config = mkIf cfg.enable {
    home.packages = [ genai-toolbox ];

    # Example configuration file for MCP Toolbox
    xdg.configFile."mcp-toolbox/config.yaml".text = ''
      # MCP Toolbox Configuration
      # See https://github.com/googleapis/genai-toolbox for details

      server:
        port: 8080
        host: "127.0.0.1"

      # Database connections
      # Add your database configurations here
      databases: []

      # Observability
      telemetry:
        enabled: true
        endpoint: "localhost:4317"
    '';
  };
}
