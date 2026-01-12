# Agenix secrets configuration
# This file defines which SSH keys can decrypt which secrets
#
# Usage:
#   1. Add your SSH public key to the appropriate list below
#   2. Create encrypted secrets with: agenix -e secrets/<name>.age
#   3. Re-key existing secrets after adding keys: agenix -r
#
# For more info: https://github.com/ryantm/agenix

let
  # =============================================================================
  # SSH Public Keys
  # =============================================================================
  # Add authorized SSH public keys here
  # These keys can decrypt the secrets they are assigned to

  # Developer keys (add your SSH public key here)
  # Example: developer1 = "ssh-ed25519 AAAA... user@host";
  developers = [
    # Add developer SSH public keys here
    # "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAI... developer@workstation"
  ];

  # CI/CD system keys
  # These keys are used by automated systems
  ci = [
    # Add CI system SSH public keys here
    # "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAI... github-actions"
  ];

  # Server/host keys (for NixOS deployments)
  # Get host key with: ssh-keyscan -t ed25519 hostname
  servers = [
    # Add server SSH host keys here
    # "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAI... root@server"
  ];

  # Combined key lists for different access levels
  allDevelopers = developers;
  allSystems = ci ++ servers;
  everyone = allDevelopers ++ allSystems;

in
{
  # =============================================================================
  # Secret Definitions
  # =============================================================================
  # Each entry maps a secret file to the keys that can decrypt it
  # Format: "secrets/<name>.age".publicKeys = [ list of authorized keys ];

  # Example secrets (uncomment and customize as needed):

  # API keys for external services
  # "openai-api-key.age".publicKeys = allDevelopers;
  # "anthropic-api-key.age".publicKeys = allDevelopers;

  # Database credentials
  # "postgres-password.age".publicKeys = everyone;

  # AGiXT configuration
  # "agixt-api-key.age".publicKeys = everyone;

  # LocalAI configuration
  # "localai-api-key.age".publicKeys = allDevelopers;

  # OAuth/OIDC secrets
  # "oauth-client-secret.age".publicKeys = everyone;
}
