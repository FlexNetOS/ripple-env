# Terraform configuration for GitHub Actions self-hosted runners with WSL2
#
# This deploys:
# - AWS EC2 instance with Windows Server 2022 + WSL2
# - Configured as a GitHub Actions self-hosted runner
# - Auto-scaling based on job queue (optional)
#
# Usage:
#   terraform init
#   terraform plan -var="github_token=YOUR_TOKEN"
#   terraform apply -var="github_token=YOUR_TOKEN"

terraform {
  required_version = ">= 1.0.0"

  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
  }

  # Optional: Use S3 backend for state
  # backend "s3" {
  #   bucket = "your-terraform-state-bucket"
  #   key    = "github-runners/terraform.tfstate"
  #   region = "us-west-2"
  # }
}

# Variables
variable "aws_region" {
  description = "AWS region for resources"
  type        = string
  default     = "us-west-2"
}

variable "github_repo" {
  description = "GitHub repository (owner/repo format)"
  type        = string
  default     = "FlexNetOS/ripple-env"
}

variable "github_token" {
  description = "GitHub personal access token or registration token"
  type        = string
  sensitive   = true
}

variable "instance_type" {
  description = "EC2 instance type (must support nested virtualization)"
  type        = string
  default     = "m5.large"
}

variable "runner_name" {
  description = "Name for the self-hosted runner"
  type        = string
  default     = "ripple-wsl2-runner"
}

variable "runner_labels" {
  description = "Labels for the runner"
  type        = list(string)
  default     = ["self-hosted", "Windows", "WSL2", "nix"]
}

variable "admin_cidr_blocks" {
  description = <<-EOT
    CIDR blocks allowed to access the instance for administrative purposes.

    Notes:
    - For production, prefer SSM Session Manager (no inbound ports required).
    - If you enable inbound access, restrict this to your corporate VPN egress.
  EOT
  type        = list(string)
  default     = []
}

variable "enable_rdp" {
  description = "Enable inbound RDP (3389) to the runner instance (NOT recommended unless restricted)."
  type        = bool
  default     = false
}

variable "enable_winrm" {
  description = "Enable inbound WinRM HTTPS (5986) to the runner instance (NOT recommended unless restricted)."
  type        = bool
  default     = false
}

variable "egress_cidr_blocks" {
  description = <<-EOT
    CIDR blocks the runner is allowed to reach (egress).

    Default is unrestricted egress, which is common for CI runners so they can
    reach GitHub and AWS endpoints. For hardened environments, restrict egress
    and route traffic via a proxy/VPC endpoints.
  EOT
  type        = list(string)
  default     = ["0.0.0.0/0"]
}

# Provider
provider "aws" {
  region = var.aws_region

  default_tags {
    tags = {
      Project     = "ripple-env"
      Environment = "ci"
      ManagedBy   = "terraform"
    }
  }
}

# Data sources
data "aws_ami" "windows_server_2022" {
  most_recent = true
  owners      = ["801119661308"] # Amazon

  filter {
    name   = "name"
    values = ["Windows_Server-2022-English-Full-Base-*"]
  }

  filter {
    name   = "virtualization-type"
    values = ["hvm"]
  }
}

data "aws_vpc" "default" {
  default = true
}

data "aws_subnets" "default" {
  filter {
    name   = "vpc-id"
    values = [data.aws_vpc.default.id]
  }
}

# Security Group
resource "aws_security_group" "runner" {
  name        = "${var.runner_name}-sg"
  description = "Security group for GitHub Actions runner"
  vpc_id      = data.aws_vpc.default.id

  # Inbound admin access is disabled by default.
  # Prefer SSM Session Manager instead of opening ports.
  dynamic "ingress" {
    for_each = var.enable_rdp && length(var.admin_cidr_blocks) > 0 ? [1] : []
    content {
      description = "RDP (restricted)"
      from_port   = 3389
      to_port     = 3389
      protocol    = "tcp"
      cidr_blocks = var.admin_cidr_blocks
    }
  }

  dynamic "ingress" {
    for_each = var.enable_winrm && length(var.admin_cidr_blocks) > 0 ? [1] : []
    content {
      description = "WinRM HTTPS (restricted)"
      from_port   = 5986
      to_port     = 5986
      protocol    = "tcp"
      cidr_blocks = var.admin_cidr_blocks
    }
  }

  # Allow all outbound (commonly required for CI runners).
  # See docs/SECURITY_SCAN_EXCEPTIONS.md for the rationale and hardening guidance.
  egress {
    description = "Runner egress"
    from_port   = 0
    to_port     = 0
    protocol    = "-1"
    cidr_blocks = var.egress_cidr_blocks
  }

  tags = {
    Name = "${var.runner_name}-sg"
  }
}

# IAM Role for EC2
resource "aws_iam_role" "runner" {
  name = "${var.runner_name}-role"

  assume_role_policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Action = "sts:AssumeRole"
        Effect = "Allow"
        Principal = {
          Service = "ec2.amazonaws.com"
        }
      }
    ]
  })
}

resource "aws_iam_role_policy_attachment" "ssm" {
  role       = aws_iam_role.runner.name
  policy_arn = "arn:aws:iam::aws:policy/AmazonSSMManagedInstanceCore"
}

resource "aws_iam_instance_profile" "runner" {
  name = "${var.runner_name}-profile"
  role = aws_iam_role.runner.name
}

# User data script to configure the runner
locals {
  user_data = <<-EOF
    <powershell>
    # Wait for network
    Start-Sleep -Seconds 30

    # Enable WSL features
    Write-Host "Enabling WSL2 features..."
    Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux -NoRestart -WarningAction SilentlyContinue
    Enable-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform -NoRestart -WarningAction SilentlyContinue

    # Download and install WSL2 kernel update
    $wslKernelUrl = 'https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi'
    Invoke-WebRequest -Uri $wslKernelUrl -OutFile 'C:\wsl_update.msi' -UseBasicParsing
    Start-Process msiexec.exe -ArgumentList '/i', 'C:\wsl_update.msi', '/quiet', '/norestart' -Wait
    Remove-Item 'C:\wsl_update.msi' -Force

    # Set WSL2 as default
    wsl --set-default-version 2

    # Install GitHub Actions runner
    Write-Host "Installing GitHub Actions runner..."
    New-Item -ItemType Directory -Path 'C:\actions-runner' -Force
    Set-Location 'C:\actions-runner'

    $runnerVersion = '2.311.0'
    $runnerUrl = "https://github.com/actions/runner/releases/download/v$runnerVersion/actions-runner-win-x64-$runnerVersion.zip"
    Invoke-WebRequest -Uri $runnerUrl -OutFile 'runner.zip' -UseBasicParsing
    Expand-Archive -Path 'runner.zip' -DestinationPath '.' -Force
    Remove-Item 'runner.zip' -Force

    # Configure and start the runner
    Write-Host "Configuring runner..."
    $labels = '${join(",", var.runner_labels)}'
    .\config.cmd --url "https://github.com/${var.github_repo}" --token "${var.github_token}" --name "${var.runner_name}" --labels $labels --unattended --replace

    # Install as service
    .\svc.cmd install
    .\svc.cmd start

    Write-Host "Runner setup complete!"

    # Schedule restart to complete WSL2 setup
    shutdown /r /t 300 /c "Restarting to complete WSL2 setup"
    </powershell>
  EOF
}

# EC2 Instance
resource "aws_instance" "runner" {
  ami                    = data.aws_ami.windows_server_2022.id
  instance_type          = var.instance_type
  subnet_id              = data.aws_subnets.default.ids[0]
  vpc_security_group_ids = [aws_security_group.runner.id]
  iam_instance_profile   = aws_iam_instance_profile.runner.name

  # Enable detailed monitoring
  monitoring = true

  # Require IMDSv2 session tokens for metadata access
  metadata_options {
    http_endpoint               = "enabled"
    http_tokens                 = "required"
    http_put_response_hop_limit = 1
  }

  # User data for setup
  user_data = base64encode(local.user_data)

  # Root volume
  root_block_device {
    volume_size           = 100
    volume_type           = "gp3"
    delete_on_termination = true
    encrypted             = true
  }

  # Prevent accidental termination
  disable_api_termination = false

  tags = {
    Name   = var.runner_name
    Runner = "github-actions"
  }

  lifecycle {
    ignore_changes = [
      user_data, # Don't recreate on user_data changes
    ]
  }
}

# Outputs
output "instance_id" {
  description = "EC2 instance ID"
  value       = aws_instance.runner.id
}

output "instance_public_ip" {
  description = "Public IP of the runner"
  value       = aws_instance.runner.public_ip
}

output "instance_private_ip" {
  description = "Private IP of the runner"
  value       = aws_instance.runner.private_ip
}

output "runner_url" {
  description = "GitHub repository runners URL"
  value       = "https://github.com/${var.github_repo}/settings/actions/runners"
}

output "rdp_connection" {
  description = "RDP connection string"
  value       = "mstsc /v:${aws_instance.runner.public_ip}"
}

output "ssm_session" {
  description = "SSM session command"
  value       = "aws ssm start-session --target ${aws_instance.runner.id}"
}
