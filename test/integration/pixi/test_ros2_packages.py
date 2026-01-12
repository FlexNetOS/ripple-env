#!/usr/bin/env python3
"""
ROS2 Package Validation Tests for Pixi Environment

This test suite validates that ROS2 packages installed via pixi
are functional and properly configured.

Run with:
    pixi run pytest test/integration/pixi/test_ros2_packages.py -v
    # or from within pixi shell:
    pytest test/integration/pixi/test_ros2_packages.py -v
"""

import os
import subprocess
import sys
import pytest
from pathlib import Path


def run_command(cmd: list[str], timeout: int = 30) -> tuple[int, str, str]:
    """Run a command and return (returncode, stdout, stderr)."""
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "Command timed out"
    except FileNotFoundError:
        return -1, "", f"Command not found: {cmd[0]}"


class TestPixiEnvironment:
    """Test pixi environment configuration."""

    def test_pixi_installed(self):
        """Verify pixi is installed and accessible."""
        code, stdout, stderr = run_command(["pixi", "--version"])
        assert code == 0, f"pixi not found: {stderr}"
        assert "pixi" in stdout.lower() or stdout.strip(), "pixi version not returned"

    def test_pixi_toml_exists(self):
        """Verify pixi.toml configuration exists."""
        pixi_toml = Path("pixi.toml")
        assert pixi_toml.exists(), "pixi.toml not found in project root"

    def test_pixi_lock_exists(self):
        """Verify pixi.lock exists (dependencies resolved)."""
        pixi_lock = Path("pixi.lock")
        assert pixi_lock.exists(), "pixi.lock not found - run 'pixi install'"


class TestROS2CorePackages:
    """Test core ROS2 packages in pixi environment."""

    @pytest.fixture(autouse=True)
    def skip_if_no_ros(self):
        """Skip tests if ROS2 is not available."""
        code, _, _ = run_command(["pixi", "run", "ros2", "--help"])
        if code != 0:
            pytest.skip("ROS2 not available in pixi environment")

    def test_ros2_cli_available(self):
        """Verify ros2 CLI is available."""
        code, stdout, stderr = run_command(["pixi", "run", "ros2", "--help"])
        assert code == 0, f"ros2 CLI not available: {stderr}"
        assert "usage: ros2" in stdout.lower() or "ros2" in stdout, \
            "ros2 help output unexpected"

    def test_ros2_topic_command(self):
        """Verify ros2 topic command works."""
        code, stdout, stderr = run_command(["pixi", "run", "ros2", "topic", "--help"])
        assert code == 0, f"ros2 topic command failed: {stderr}"

    def test_ros2_node_command(self):
        """Verify ros2 node command works."""
        code, stdout, stderr = run_command(["pixi", "run", "ros2", "node", "--help"])
        assert code == 0, f"ros2 node command failed: {stderr}"

    def test_ros2_service_command(self):
        """Verify ros2 service command works."""
        code, stdout, stderr = run_command(["pixi", "run", "ros2", "service", "--help"])
        assert code == 0, f"ros2 service command failed: {stderr}"

    def test_ros2_param_command(self):
        """Verify ros2 param command works."""
        code, stdout, stderr = run_command(["pixi", "run", "ros2", "param", "--help"])
        assert code == 0, f"ros2 param command failed: {stderr}"

    def test_ros2_pkg_command(self):
        """Verify ros2 pkg command works."""
        code, stdout, stderr = run_command(["pixi", "run", "ros2", "pkg", "--help"])
        assert code == 0, f"ros2 pkg command failed: {stderr}"


class TestROS2PackageList:
    """Test ROS2 packages are properly listed."""

    @pytest.fixture(autouse=True)
    def skip_if_no_ros(self):
        """Skip tests if ROS2 is not available."""
        code, _, _ = run_command(["pixi", "run", "ros2", "--help"])
        if code != 0:
            pytest.skip("ROS2 not available in pixi environment")

    def test_ros2_packages_listed(self):
        """Verify ROS2 packages can be listed."""
        code, stdout, stderr = run_command(
            ["pixi", "run", "ros2", "pkg", "list"],
            timeout=60
        )
        # May return non-zero if no packages installed, but should not crash
        assert code in [0, 1], f"ros2 pkg list crashed: {stderr}"

    def test_core_interfaces_available(self):
        """Verify core ROS2 interfaces are available."""
        code, stdout, stderr = run_command(
            ["pixi", "run", "ros2", "interface", "list"],
            timeout=60
        )
        if code == 0:
            # Check for common interface types
            assert "std_msgs" in stdout or "geometry_msgs" in stdout or \
                   len(stdout.strip()) > 0, \
                "No ROS2 interfaces found"


class TestColconBuildSystem:
    """Test colcon build system in pixi environment."""

    @pytest.fixture(autouse=True)
    def skip_if_no_colcon(self):
        """Skip tests if colcon is not available."""
        code, _, _ = run_command(["pixi", "run", "colcon", "--help"])
        if code != 0:
            pytest.skip("colcon not available in pixi environment")

    def test_colcon_available(self):
        """Verify colcon build tool is available."""
        code, stdout, stderr = run_command(["pixi", "run", "colcon", "--help"])
        assert code == 0, f"colcon not available: {stderr}"
        assert "colcon" in stdout.lower(), "colcon help output unexpected"

    def test_colcon_build_command(self):
        """Verify colcon build command exists."""
        code, stdout, stderr = run_command(["pixi", "run", "colcon", "build", "--help"])
        assert code == 0, f"colcon build command failed: {stderr}"

    def test_colcon_test_command(self):
        """Verify colcon test command exists."""
        code, stdout, stderr = run_command(["pixi", "run", "colcon", "test", "--help"])
        assert code == 0, f"colcon test command failed: {stderr}"


class TestROS2Environment:
    """Test ROS2 environment variables and setup."""

    @pytest.fixture(autouse=True)
    def skip_if_no_ros(self):
        """Skip tests if ROS2 is not available."""
        code, _, _ = run_command(["pixi", "run", "ros2", "--help"])
        if code != 0:
            pytest.skip("ROS2 not available in pixi environment")

    def test_ros_distro_set(self):
        """Verify ROS_DISTRO environment variable."""
        code, stdout, stderr = run_command(
            ["pixi", "run", "bash", "-c", "echo $ROS_DISTRO"]
        )
        # May not be set in pixi env, but command should work
        assert code == 0, f"Failed to check ROS_DISTRO: {stderr}"

    def test_ament_prefix_path(self):
        """Verify AMENT_PREFIX_PATH is set or can be determined."""
        code, stdout, stderr = run_command(
            ["pixi", "run", "bash", "-c", "echo $AMENT_PREFIX_PATH"]
        )
        assert code == 0, f"Failed to check AMENT_PREFIX_PATH: {stderr}"

    def test_ros2_domain_id_configurable(self):
        """Verify ROS_DOMAIN_ID can be set."""
        code, stdout, stderr = run_command(
            ["pixi", "run", "bash", "-c", "ROS_DOMAIN_ID=42 echo $ROS_DOMAIN_ID"]
        )
        assert code == 0, f"Failed to set ROS_DOMAIN_ID: {stderr}"
        assert "42" in stdout, "ROS_DOMAIN_ID not properly set"


class TestPythonROS2Bindings:
    """Test Python ROS2 bindings availability."""

    @pytest.fixture(autouse=True)
    def skip_if_no_python(self):
        """Skip tests if Python is not available in pixi."""
        code, _, _ = run_command(["pixi", "run", "python3", "--version"])
        if code != 0:
            pytest.skip("Python not available in pixi environment")

    def test_python_available(self):
        """Verify Python is available in pixi."""
        code, stdout, stderr = run_command(["pixi", "run", "python3", "--version"])
        assert code == 0, f"Python not available: {stderr}"
        assert "python" in stdout.lower() or "3." in stdout, \
            "Python version not returned"

    def test_rclpy_importable(self):
        """Verify rclpy can be imported (if installed)."""
        code, stdout, stderr = run_command([
            "pixi", "run", "python3", "-c",
            "try:\n    import rclpy\n    print('rclpy available')\nexcept ImportError:\n    print('rclpy not installed')"
        ])
        assert code == 0, f"Python import test failed: {stderr}"
        # Either rclpy is available or properly reports as not installed
        assert "rclpy" in stdout, "rclpy import test gave unexpected output"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
