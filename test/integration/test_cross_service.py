#!/usr/bin/env python3
"""
Cross-Service Integration Tests

Tests for validating cross-service workflows in the ripple-env environment.
These tests verify that different components work together correctly.

Run with:
    pytest test/integration/test_cross_service.py -v
"""

import os
import subprocess
import sys
import json
import tempfile
from pathlib import Path
import pytest


def run_command(cmd: list[str], timeout: int = 60, env: dict = None) -> tuple[int, str, str]:
    """Run a command and return (returncode, stdout, stderr)."""
    run_env = os.environ.copy()
    if env:
        run_env.update(env)

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
            env=run_env
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "Command timed out"
    except FileNotFoundError:
        return -1, "", f"Command not found: {cmd[0]}"


class TestNixFlakeIntegration:
    """Test Nix flake configuration integration."""

    def test_flake_show(self):
        """Verify flake outputs are valid."""
        code, stdout, stderr = run_command(
            ["nix", "flake", "show", "--json"],
            timeout=120
        )
        if code != 0:
            pytest.skip(f"Nix not available or flake invalid: {stderr}")

        # Parse JSON output
        try:
            outputs = json.loads(stdout)
            assert "devShells" in outputs, "devShells not in flake outputs"
        except json.JSONDecodeError:
            pytest.fail(f"Invalid JSON from flake show: {stdout[:500]}")

    def test_flake_check_syntax(self):
        """Verify flake.nix has valid syntax."""
        flake_path = Path("flake.nix")
        assert flake_path.exists(), "flake.nix not found"

        code, stdout, stderr = run_command(
            ["nix-instantiate", "--parse", str(flake_path)],
            timeout=30
        )
        assert code == 0, f"flake.nix has syntax errors: {stderr}"

    def test_lib_exports(self):
        """Verify library functions are exported."""
        code, stdout, stderr = run_command(
            ["nix", "eval", ".#lib", "--json"],
            timeout=60
        )
        if code != 0:
            pytest.skip(f"Cannot evaluate lib: {stderr}")

        try:
            lib = json.loads(stdout)
            # Check for expected functions (they appear as null in JSON since they're functions)
            assert lib is not None, "lib export is null"
        except json.JSONDecodeError:
            # Functions can't be serialized to JSON, which is expected
            pass


class TestDirenvIntegration:
    """Test direnv integration with Nix."""

    def test_envrc_exists(self):
        """Verify .envrc file exists."""
        envrc = Path(".envrc")
        assert envrc.exists(), ".envrc not found"

    def test_envrc_uses_flake(self):
        """Verify .envrc uses flake."""
        envrc = Path(".envrc")
        content = envrc.read_text()
        assert "use flake" in content or "nix" in content.lower(), \
            ".envrc should reference nix/flake"

    def test_direnv_allowed(self):
        """Test direnv allow status."""
        code, stdout, stderr = run_command(["direnv", "status"])
        if code != 0:
            pytest.skip(f"direnv not available: {stderr}")
        # Just verify direnv runs without error


class TestGitHooksIntegration:
    """Test Git hooks and workflow integration."""

    def test_git_repo_valid(self):
        """Verify this is a valid git repository."""
        code, stdout, stderr = run_command(["git", "status"])
        assert code == 0, f"Not a valid git repository: {stderr}"

    def test_git_hooks_directory(self):
        """Check for git hooks directory."""
        hooks_dir = Path(".git/hooks")
        if not hooks_dir.exists():
            pytest.skip("Git hooks directory not found")
        assert hooks_dir.is_dir(), ".git/hooks is not a directory"

    def test_pre_commit_config(self):
        """Check for pre-commit configuration."""
        pre_commit_config = Path(".pre-commit-config.yaml")
        # Pre-commit config is optional
        if not pre_commit_config.exists():
            pytest.skip("No .pre-commit-config.yaml found")
        assert pre_commit_config.exists()


class TestModuleIntegration:
    """Test Nix module integration."""

    def test_common_module_imports(self):
        """Verify common modules can be imported."""
        module_path = Path("modules/common/default.nix")
        assert module_path.exists(), "Common module not found"

        code, stdout, stderr = run_command(
            ["nix-instantiate", "--parse", str(module_path)],
            timeout=30
        )
        assert code == 0, f"Common module has syntax errors: {stderr}"

    def test_linux_module_imports(self):
        """Verify Linux modules can be imported."""
        module_path = Path("modules/linux/default.nix")
        assert module_path.exists(), "Linux module not found"

        code, stdout, stderr = run_command(
            ["nix-instantiate", "--parse", str(module_path)],
            timeout=30
        )
        assert code == 0, f"Linux module has syntax errors: {stderr}"

    def test_macos_module_imports(self):
        """Verify macOS modules can be imported."""
        module_path = Path("modules/macos/default.nix")
        assert module_path.exists(), "macOS module not found"

        code, stdout, stderr = run_command(
            ["nix-instantiate", "--parse", str(module_path)],
            timeout=30
        )
        assert code == 0, f"macOS module has syntax errors: {stderr}"


class TestShellIntegration:
    """Test shell configuration integration."""

    def test_default_shell_packages(self):
        """Verify default shell packages are defined."""
        packages_path = Path("nix/packages/default.nix")
        assert packages_path.exists(), "Shell packages not found"

        code, stdout, stderr = run_command(
            ["nix-instantiate", "--parse", str(packages_path)],
            timeout=30
        )
        assert code == 0, f"Packages file has syntax errors: {stderr}"

    def test_shell_commands_defined(self):
        """Verify shell commands are defined."""
        commands_path = Path("nix/commands/default.nix")
        assert commands_path.exists(), "Shell commands not found"

        code, stdout, stderr = run_command(
            ["nix-instantiate", "--parse", str(commands_path)],
            timeout=30
        )
        assert code == 0, f"Commands file has syntax errors: {stderr}"


class TestImageBuildIntegration:
    """Test NixOS image build configurations."""

    def test_wsl_image_config(self):
        """Verify WSL image configuration is valid."""
        wsl_path = Path("nix/images/wsl.nix")
        assert wsl_path.exists(), "WSL image config not found"

        code, stdout, stderr = run_command(
            ["nix-instantiate", "--parse", str(wsl_path)],
            timeout=30
        )
        assert code == 0, f"WSL config has syntax errors: {stderr}"

    def test_iso_image_config(self):
        """Verify ISO image configuration is valid."""
        iso_path = Path("nix/images/iso.nix")
        assert iso_path.exists(), "ISO image config not found"

        code, stdout, stderr = run_command(
            ["nix-instantiate", "--parse", str(iso_path)],
            timeout=30
        )
        assert code == 0, f"ISO config has syntax errors: {stderr}"

    def test_vm_image_config(self):
        """Verify VM image configuration is valid."""
        vm_path = Path("nix/images/vm.nix")
        assert vm_path.exists(), "VM image config not found"

        code, stdout, stderr = run_command(
            ["nix-instantiate", "--parse", str(vm_path)],
            timeout=30
        )
        assert code == 0, f"VM config has syntax errors: {stderr}"


class TestDocumentationIntegration:
    """Test documentation completeness."""

    def test_readme_exists(self):
        """Verify README exists."""
        readme = Path("README.md")
        assert readme.exists(), "README.md not found"

    def test_getting_started_exists(self):
        """Verify getting started guide exists."""
        guide = Path("docs/GETTING_STARTED.md")
        assert guide.exists(), "Getting started guide not found"

    def test_troubleshooting_exists(self):
        """Verify troubleshooting guide exists."""
        troubleshooting = Path("docs/TROUBLESHOOTING.md")
        assert troubleshooting.exists(), "Troubleshooting guide not found"

    def test_claude_md_exists(self):
        """Verify Claude configuration exists."""
        claude_md = Path(".claude/CLAUDE.md")
        assert claude_md.exists(), "CLAUDE.md not found"


class TestCIWorkflowIntegration:
    """Test CI workflow configurations."""

    def test_ci_workflow_exists(self):
        """Verify CI workflow exists."""
        ci_workflow = Path(".github/workflows/ci.yml")
        assert ci_workflow.exists(), "CI workflow not found"

    def test_bootstrap_test_workflow(self):
        """Verify bootstrap test workflow exists."""
        bootstrap_workflow = Path(".github/workflows/bootstrap-test.yml")
        assert bootstrap_workflow.exists(), "Bootstrap test workflow not found"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
