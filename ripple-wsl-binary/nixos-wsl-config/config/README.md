# WSL Config Directory

This directory contains configuration files for the FlexNetOS WSL distribution.

## Docker Configuration

The `docker/` directory is a symlink to the main `docker/` directory at the repository root to avoid configuration drift. All docker-compose files are maintained in a single location.

When building the WSL distribution, the build scripts will copy the docker configs from this symlinked location to `/opt/flexnetos/config/docker/` in the deployed system.

## Updating Docker Configs

To update docker configurations:
1. Edit files in the main `docker/` directory at the repository root
2. Changes will automatically be included when building the WSL distribution
