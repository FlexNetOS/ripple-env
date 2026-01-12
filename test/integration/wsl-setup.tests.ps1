<#
.SYNOPSIS
    Integration tests for WSL2 setup

.DESCRIPTION
    These tests run on actual WSL2 environments.
    Requires WSL2 to be installed and running.

.NOTES
    Run with: Invoke-Pester -Path test/integration/wsl-setup.tests.ps1 -CI
#>

Describe "WSL2 Environment" -Tag "Integration" {
    BeforeAll {
        $wslInstalled = $false
        try {
            $wslStatus = wsl --status 2>&1
            if ($LASTEXITCODE -eq 0) {
                $wslInstalled = $true
            }
        }
        catch {
            $wslInstalled = $false
        }
    }

    It "Should have WSL installed" {
        $wslInstalled | Should -Be $true
    }

    It "Should have WSL2 as default version" -Skip:(-not $wslInstalled) {
        $status = wsl --status 2>&1 | Out-String
        $status | Should -Match "Default Version: 2|既定のバージョン: 2"
    }

    It "Should be able to run basic Linux commands" -Skip:(-not $wslInstalled) {
        $result = wsl -- echo "hello"
        $result | Should -Be "hello"
    }
}

Describe "NixOS Distribution" -Tag "Integration", "NixOS" {
    BeforeAll {
        $distroName = "NixOS-Ripple-Test"
        $distroExists = $false

        try {
            $distros = wsl --list --quiet 2>&1
            $distroExists = $distros -contains $distroName
        }
        catch {
            $distroExists = $false
        }
    }

    It "Should have NixOS-Ripple-Test distribution" -Skip:(-not $distroExists) {
        $distros = wsl --list --quiet
        $distros | Should -Contain "NixOS-Ripple-Test"
    }

    It "Should have Nix installed in the distribution" -Skip:(-not $distroExists) {
        $result = wsl -d NixOS-Ripple-Test -- nix --version 2>&1
        $LASTEXITCODE | Should -Be 0
        $result | Should -Match "nix"
    }

    It "Should have flakes enabled" -Skip:(-not $distroExists) {
        $result = wsl -d NixOS-Ripple-Test -- nix flake --help 2>&1
        $LASTEXITCODE | Should -Be 0
    }
}

Describe "Development Environment" -Tag "Integration", "DevEnv" {
    BeforeAll {
        $distroName = "NixOS-Ripple-Test"
        $repoExists = $false

        try {
            $result = wsl -d $distroName -- bash -c "test -d ~/ripple-env && echo 'exists'"
            $repoExists = $result -eq "exists"
        }
        catch {
            $repoExists = $false
        }
    }

    It "Should have ripple-env repository cloned" -Skip:(-not $repoExists) {
        $result = wsl -d NixOS-Ripple-Test -- bash -c "test -d ~/ripple-env && echo 'exists'"
        $result | Should -Be "exists"
    }

    It "Should have valid flake.nix" -Skip:(-not $repoExists) {
        $result = wsl -d NixOS-Ripple-Test -- bash -c "test -f ~/ripple-env/flake.nix && echo 'exists'"
        $result | Should -Be "exists"
    }

    It "Should have pixi.toml" -Skip:(-not $repoExists) {
        $result = wsl -d NixOS-Ripple-Test -- bash -c "test -f ~/ripple-env/pixi.toml && echo 'exists'"
        $result | Should -Be "exists"
    }

    It "Should be able to enter dev shell" -Skip:(-not $repoExists) {
        $result = wsl -d NixOS-Ripple-Test -- bash -c "cd ~/ripple-env && nix develop --command echo 'shell works'"
        $result | Should -Contain "shell works"
    }
}

Describe "Cleanup" -Tag "Integration", "Cleanup" {
    It "Should be able to shutdown WSL" {
        wsl --shutdown
        Start-Sleep -Seconds 2
        # WSL should restart on next command
        $result = wsl -- echo "restarted"
        $result | Should -Be "restarted"
    }
}
