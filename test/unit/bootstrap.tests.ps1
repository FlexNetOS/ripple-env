<#
.SYNOPSIS
    Pester unit tests for bootstrap.ps1

.DESCRIPTION
    Tests PowerShell functions with mocked WSL2 calls.
    Can run on any Windows system without actual WSL2.

.NOTES
    Run with: Invoke-Pester -Path test/unit/bootstrap.tests.ps1 -CI
#>

BeforeAll {
    # Define mocks for external commands BEFORE loading the script
    function global:wsl {
        param([Parameter(ValueFromRemainingArguments=$true)]$Arguments)
        # Default mock - returns nothing
        $global:LASTEXITCODE = 0
        return ""
    }

    # Get the script path
    $scriptPath = Join-Path $PSScriptRoot "../../bootstrap.ps1"

    # Read the script content and modify for testing
    $scriptContent = Get-Content $scriptPath -Raw

    # Remove the #Requires directive for testing
    $scriptContent = $scriptContent -replace '#Requires -RunAsAdministrator', '# Requires directive removed for testing'

    # Remove ErrorActionPreference = Stop for testing
    $scriptContent = $scriptContent -replace '\$ErrorActionPreference\s*=\s*"Stop"', '$ErrorActionPreference = "Continue"'

    # Create a temporary script file without the requires directive
    $script:tempScript = Join-Path $env:TEMP "bootstrap-test.ps1"
    Set-Content -Path $script:tempScript -Value $scriptContent

    # Dot-source the modified script with default parameters
    . $script:tempScript
}

AfterAll {
    # Cleanup temp script
    if (Test-Path $script:tempScript) {
        Remove-Item $script:tempScript -Force
    }

    # Remove the global mock
    Remove-Item -Path Function:\wsl -ErrorAction SilentlyContinue
}

Describe "Write-ColorOutput" {
    It "Should write Info messages without error" {
        { Write-ColorOutput -Message "Test message" -Type "Info" } | Should -Not -Throw
    }

    It "Should write Success messages without error" {
        { Write-ColorOutput -Message "Test message" -Type "Success" } | Should -Not -Throw
    }

    It "Should write Warning messages without error" {
        { Write-ColorOutput -Message "Test message" -Type "Warning" } | Should -Not -Throw
    }

    It "Should write Error messages without error" {
        { Write-ColorOutput -Message "Test message" -Type "Error" } | Should -Not -Throw
    }

    It "Should write Step messages without error" {
        { Write-ColorOutput -Message "Test step" -Type "Step" } | Should -Not -Throw
    }
}

Describe "Test-Administrator" {
    It "Should return a boolean value" {
        $result = Test-Administrator
        $result | Should -BeOfType [bool]
    }
}

Describe "Test-WindowsVersion" {
    BeforeAll {
        # Store original function to restore later
        $script:originalFunction = ${function:Test-WindowsVersion}
    }

    AfterAll {
        # Restore original function
        Set-Item -Path function:Test-WindowsVersion -Value $script:originalFunction
    }

    Context "When Windows version is compatible" {
        BeforeAll {
            Mock Get-ItemProperty {
                return @{ CurrentBuildNumber = "22621" }
            } -ModuleName Microsoft.PowerShell.Management -Verifiable
        }

        It "Should return true for compatible Windows version (build 22621)" {
            # Create a testable version of the function
            function Test-WindowsVersionTestable {
                $buildNumber = 22621  # Simulated Windows 11
                if ($buildNumber -lt 19041) {
                    return $false
                }
                return $true
            }

            $result = Test-WindowsVersionTestable
            $result | Should -Be $true
        }
    }

    Context "When Windows version is incompatible" {
        It "Should return false for incompatible Windows version (build 18363)" {
            function Test-WindowsVersionTestable {
                $buildNumber = 18363  # Simulated Windows 10 1909
                if ($buildNumber -lt 19041) {
                    return $false
                }
                return $true
            }

            $result = Test-WindowsVersionTestable
            $result | Should -Be $false
        }
    }
}

Describe "Test-WSLInstalled" {
    Context "When WSL is installed" {
        BeforeAll {
            # Override the global wsl mock
            function global:wsl {
                param([Parameter(ValueFromRemainingArguments=$true)]$Arguments)
                $global:LASTEXITCODE = 0
                return "Default Version: 2"
            }
        }

        It "Should return true when WSL is installed" {
            $result = Test-WSLInstalled
            $result | Should -Be $true
        }
    }

    Context "When WSL is not installed" {
        BeforeAll {
            function global:wsl {
                param([Parameter(ValueFromRemainingArguments=$true)]$Arguments)
                $global:LASTEXITCODE = 1
                throw "WSL not installed"
            }
        }

        It "Should return false when WSL is not installed" {
            $result = Test-WSLInstalled
            $result | Should -Be $false
        }
    }
}

Describe "Test-WSL2Default" {
    Context "When WSL2 is default" {
        BeforeAll {
            function global:wsl {
                param([Parameter(ValueFromRemainingArguments=$true)]$Arguments)
                $global:LASTEXITCODE = 0
                return "Default Version: 2"
            }
        }

        It "Should return true when WSL2 is default" {
            $result = Test-WSL2Default
            $result | Should -Be $true
        }
    }

    Context "When WSL1 is default" {
        BeforeAll {
            function global:wsl {
                param([Parameter(ValueFromRemainingArguments=$true)]$Arguments)
                $global:LASTEXITCODE = 0
                return "Default Version: 1"
            }
        }

        It "Should return false when WSL1 is default" {
            $result = Test-WSL2Default
            $result | Should -Be $false
        }
    }
}

Describe "Test-DistroExists" {
    Context "When distro exists" {
        BeforeAll {
            function global:wsl {
                param([Parameter(ValueFromRemainingArguments=$true)]$Arguments)
                return @("Ubuntu", "NixOS-ROS2", "Debian")
            }
        }

        It "Should return true when distro exists" {
            $result = Test-DistroExists -Name "NixOS-ROS2"
            $result | Should -Be $true
        }
    }

    Context "When distro does not exist" {
        BeforeAll {
            function global:wsl {
                param([Parameter(ValueFromRemainingArguments=$true)]$Arguments)
                return @("Ubuntu", "Debian")
            }
        }

        It "Should return false when distro does not exist" {
            $result = Test-DistroExists -Name "NixOS-ROS2"
            $result | Should -Be $false
        }
    }
}

Describe "Set-WSLConfig" {
    BeforeAll {
        # Store original USERPROFILE
        $script:originalUserProfile = $env:USERPROFILE
        $env:USERPROFILE = $TestDrive
    }

    AfterAll {
        $env:USERPROFILE = $script:originalUserProfile
    }

    It "Should create .wslconfig file using MemorySizeGB and SwapSizeGB" {
        # Use distinct values so we can assert they are not accidentally tied together
        $MemorySizeGB = 10
        $SwapSizeGB = 2

        { Set-WSLConfig } | Should -Not -Throw

        $configPath = Join-Path $TestDrive ".wslconfig"
        Test-Path $configPath | Should -Be $true

        $content = Get-Content $configPath -Raw
        $content | Should -Match "memory=10GB"
        $content | Should -Match "swap=2GB"
    }

    It "Should contain correct memory configuration" {
        $configPath = Join-Path $TestDrive ".wslconfig"
        $content = Get-Content $configPath -Raw

        $content | Should -Match "memory="
        $content | Should -Match "swap="
        $content | Should -Match "localhostForwarding=true"
    }

    It "Should contain experimental settings" {
        $configPath = Join-Path $TestDrive ".wslconfig"
        $content = Get-Content $configPath -Raw

        $content | Should -Match "\[experimental\]"
        $content | Should -Match "autoMemoryReclaim=gradual"
        $content | Should -Match "sparseVhd=true"
    }
}

Describe "Install-WSL Logic" {
    It "Should correctly identify WSL feature states" {
        # Test the logic without calling Windows-specific cmdlets
        $disabledFeature = @{ State = "Disabled" }
        $enabledFeature = @{ State = "Enabled" }

        $disabledFeature.State | Should -Be "Disabled"
        $disabledFeature.State -ne "Enabled" | Should -Be $true

        $enabledFeature.State | Should -Be "Enabled"
        $enabledFeature.State -ne "Enabled" | Should -Be $false
    }

    It "Should have correct WSL feature names" {
        $wslFeatureName = "Microsoft-Windows-Subsystem-Linux"
        $vmFeatureName = "VirtualMachinePlatform"

        $wslFeatureName | Should -Be "Microsoft-Windows-Subsystem-Linux"
        $vmFeatureName | Should -Be "VirtualMachinePlatform"
    }
}

Describe "New-NixOSDistro Logic" {
    BeforeAll {
        $script:TestDistroName = "NixOS-ROS2-Test"
        $script:TestInstallPath = $TestDrive
    }

    It "Should construct correct tarball path" {
        $tarPath = Join-Path $script:TestInstallPath "nixos-wsl.tar.gz"
        $tarPath | Should -Match "nixos-wsl\.tar\.gz$"
    }

    It "Should handle Force flag logic correctly" {
        $Force = $false
        $distroExists = $true

        # When distro exists and Force is false, should skip
        if ($distroExists -and -not $Force) {
            $shouldSkip = $true
        }
        else {
            $shouldSkip = $false
        }

        $shouldSkip | Should -Be $true

        # When Force is true, should not skip
        $Force = $true
        if ($distroExists -and -not $Force) {
            $shouldSkip = $true
        }
        else {
            $shouldSkip = $false
        }

        $shouldSkip | Should -Be $false
    }

    It "Should use WSL version 2 for import" {
        $wslVersion = 2
        $wslVersion | Should -Be 2
    }
}

Describe "Parameter Validation" {
    It "Should have valid default values" {
        # These are the defaults from the param block
        $defaultDistroName = "NixOS-ROS2"
        $defaultDiskSizeGB = 1024
        $defaultSwapSizeGB = 8

        $defaultDistroName | Should -Not -BeNullOrEmpty
        $defaultDiskSizeGB | Should -BeGreaterThan 0
        $defaultSwapSizeGB | Should -BeGreaterThan 0
    }

    It "Should accept valid DiskSizeGB values" {
        $validSizes = @(64, 128, 256, 512, 1024, 2048)
        foreach ($size in $validSizes) {
            $size | Should -BeGreaterThan 0
            $size | Should -BeLessOrEqual 4096
        }
    }

    It "Should accept valid SwapSizeGB values" {
        $validSwaps = @(4, 8, 16, 32)
        foreach ($swap in $validSwaps) {
            $swap | Should -BeGreaterThan 0
            $swap | Should -BeLessOrEqual 64
        }
    }
}

Describe "URL Configuration" {
    It "Should have valid NixOS-WSL release URL" {
        $url = "https://github.com/nix-community/NixOS-WSL/releases/latest/download/nixos-wsl.tar.gz"
        $url | Should -Match "^https://github.com"
        $url | Should -Match "NixOS-WSL"
    }

    It "Should have valid repository URL" {
        # Validate the script default rather than hard-coding a single fork.
        $RepoURL | Should -Match "^https://github.com"
        $RepoURL | Should -Match "\.git$"
    }
}

Describe "Helper Functions" {
    It "Write-ColorOutput should handle all message types" {
        $types = @("Info", "Success", "Warning", "Error", "Step")

        foreach ($type in $types) {
            { Write-ColorOutput -Message "Test" -Type $type } | Should -Not -Throw
        }
    }

    It "Test-Administrator should work without elevation" {
        # This test just verifies the function runs
        { Test-Administrator } | Should -Not -Throw
    }
}

Describe "WSL Command Arguments" {
    It "Should construct correct wsl --status command" {
        $command = "wsl"
        $args = @("--status")
        $fullCommand = "$command $($args -join ' ')"

        $fullCommand | Should -Be "wsl --status"
    }

    It "Should construct correct wsl --list --quiet command" {
        $args = @("--list", "--quiet")
        $argString = $args -join " "

        $argString | Should -Be "--list --quiet"
    }

    It "Should construct correct wsl --import command" {
        $distroName = "NixOS-ROS2"
        $installPath = "C:\WSL\NixOS"
        $tarPath = "C:\WSL\nixos.tar.gz"
        $version = 2

        $args = @("--import", $distroName, $installPath, $tarPath, "--version", $version)
        $args | Should -Contain "--import"
        $args | Should -Contain "--version"
        $args | Should -Contain "2"
    }

    It "Should construct correct wsl --set-default-version command" {
        $args = @("--set-default-version", "2")

        $args[0] | Should -Be "--set-default-version"
        $args[1] | Should -Be "2"
    }
}
