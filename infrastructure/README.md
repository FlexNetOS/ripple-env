# Infrastructure as Code

This directory contains infrastructure-as-code for deploying GitHub Actions self-hosted runners with WSL2 support.

## Overview

The bootstrap.ps1 script requires WSL2 for end-to-end testing. Since GitHub-hosted runners don't have WSL2 pre-configured, we need self-hosted runners. This infrastructure provides two deployment options:

1. **Packer AMI** - Build a custom Windows AMI with WSL2 pre-installed
2. **Terraform Deployment** - Deploy the runner to AWS EC2

## Quick Start

### Option 1: Use Pre-built AMI with Terraform

```bash
cd terraform

# Initialize Terraform
terraform init

# Get a runner registration token from:
# https://github.com/FlexNetOS/ripple-env/settings/actions/runners/new

# Deploy the runner
terraform plan -var="github_token=YOUR_REGISTRATION_TOKEN"
terraform apply -var="github_token=YOUR_REGISTRATION_TOKEN"
```

### Option 2: Build Custom AMI First

```bash
cd packer

# Initialize Packer
packer init windows-wsl2-runner.pkr.hcl

# Build the AMI
packer build windows-wsl2-runner.pkr.hcl

# Then use Terraform with the custom AMI
cd ../terraform
terraform apply -var="ami_id=ami-XXXXXXXXX" -var="github_token=YOUR_TOKEN"
```

## Components

### Packer (`packer/`)

- `windows-wsl2-runner.pkr.hcl` - Builds Windows Server 2022 AMI with:
  - WSL2 enabled and configured
  - GitHub Actions runner pre-installed
  - NixOS-WSL pre-downloaded
  - Optimized Windows settings for CI

### Terraform (`terraform/`)

- `main.tf` - Deploys:
  - EC2 instance (m5.large by default)
  - Security group with RDP/WinRM access
  - IAM role with SSM access
  - Automatic runner registration

## Requirements

### AWS
- AWS account with EC2 permissions
- AWS CLI configured (`aws configure`)
- Terraform >= 1.0.0
- Packer >= 1.8.0

### GitHub
- Repository admin access
- Personal access token or runner registration token

## Cost Estimate

| Resource | Cost |
|----------|------|
| m5.large (on-demand) | ~$0.096/hour |
| 100GB gp3 EBS | ~$8/month |
| Data transfer | Varies |

**Estimated monthly cost**: ~$80-150 (if running 24/7)

**For CI only**: Use spot instances or scheduled scaling to reduce costs.

## Security Considerations

1. **Restrict Security Group** - The default config allows RDP from anywhere. Restrict to your IP in production.

2. **Use Secrets Manager** - Store the GitHub token in AWS Secrets Manager instead of passing as variable.

3. **IAM Least Privilege** - The default IAM role has SSM access. Adjust based on needs.

4. **Encryption** - EBS volumes are encrypted by default.

## Troubleshooting

### Runner not appearing in GitHub

1. Check the instance is running: `aws ec2 describe-instances`
2. Connect via RDP or SSM: `aws ssm start-session --target i-xxx`
3. Check runner logs: `C:\actions-runner\_diag\`

### WSL2 not working

1. Ensure the instance has rebooted after initial setup
2. Check WSL status: `wsl --status`
3. Update WSL kernel: `wsl --update`

### Performance issues

1. Use m5.xlarge or larger for better performance
2. Enable Enhanced Networking
3. Use gp3 volumes with higher IOPS

## Cleanup

```bash
# Remove the runner from GitHub first, then:
terraform destroy
```

## Alternative: Use terraform-aws-github-runner module

For production deployments with auto-scaling, consider the [terraform-aws-github-runner](https://github.com/philips-labs/terraform-aws-github-runner) module.

```hcl
module "runners" {
  source  = "philips-labs/github-runner/aws"
  version = "~> 5.0"

  github_app = {
    key_base64     = var.github_app_key_base64
    id             = var.github_app_id
    webhook_secret = var.github_webhook_secret
  }

  runner_os = "windows"
  runner_architecture = "x64"

  # Custom AMI with WSL2
  ami_filter = {
    name = ["github-runner-windows-wsl2-*"]
  }
}
```
