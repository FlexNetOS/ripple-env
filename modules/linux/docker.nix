# Docker configuration for Linux
# Container support for ROS2 development
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkDefault;
in
{
  home.packages = with pkgs; [
    docker-compose     # Docker compose tool
    dive               # Docker image explorer
    lazydocker         # Docker TUI
  ];

  # Docker aliases
  home.shellAliases = {
    "dc" = "docker compose";
    "dcu" = "docker compose up -d";
    "dcd" = "docker compose down";
    "dcl" = "docker compose logs -f";
    "dps" = "docker ps";
    "dpsa" = "docker ps -a";
    "dimg" = "docker images";
    "drm" = "docker rm";
    "drmi" = "docker rmi";
  };

  # Docker environment
  home.sessionVariables = {
    # Use buildkit for faster builds
    DOCKER_BUILDKIT = "1";
    COMPOSE_DOCKER_CLI_BUILD = "1";
  };
}
