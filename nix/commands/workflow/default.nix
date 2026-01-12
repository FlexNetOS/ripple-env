# Workflow commands aggregator
# Imports all workflow-related command modules
#
# Usage in commands/default.nix:
#   workflow = import ./workflow { inherit pkgs; };
{ pkgs, ... }:

let
  # Import individual workflow modules
  github = import ./github.nix { inherit pkgs; };
  temporal = import ./temporal.nix { inherit pkgs; };
  n8n = import ./n8n.nix { inherit pkgs; };
  status = import ./status.nix { inherit pkgs; };

  # db-query and codebase-db remain in the parent workflow.nix
  # as they are larger and more complex tools
in
{
  # Individual module exports for selective inclusion
  inherit github temporal n8n status;

  # Combined list for full workflow shell
  all = github ++ temporal ++ n8n ++ status;

  # Minimal workflow commands (status only)
  minimal = status;

  # GitHub-focused commands
  githubTools = github ++ status;

  # Orchestration commands (Temporal + n8n)
  orchestration = temporal ++ n8n ++ status;
}
