#!/usr/bin/env bash
# check-python-deps.sh - Python dependency conflict detection and resolution
#
# Usage:
#   ./scripts/check-python-deps.sh [environment]
#
# Examples:
#   ./scripts/check-python-deps.sh          # Check default environment
#   ./scripts/check-python-deps.sh cuda     # Check CUDA environment
#   ./scripts/check-python-deps.sh aios     # Check AIOS environment
#   ./scripts/check-python-deps.sh --all    # Check all environments
#
# Exit codes:
#   0 - No conflicts detected
#   1 - Conflicts detected (see output for details)
#   2 - Script error

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default environment
ENV="${1:-default}"

# Print functions
info() { echo -e "${BLUE}[INFO]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }
success() { echo -e "${GREEN}[OK]${NC} $*"; }

# Track issues
ISSUES=0

# Check if pixi is available
check_pixi() {
    if ! command -v pixi &> /dev/null; then
        error "pixi not found. Please run from a Nix shell: nix develop"
        exit 2
    fi
}

# Check Python version constraints
check_python_version() {
    local env="$1"
    info "Checking Python version for environment: $env"

    local python_version
    python_version=$(pixi run -e "$env" python -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')" 2>/dev/null || echo "unknown")

    case "$env" in
        aios|aios-cuda)
            # AIOS requires Python 3.10-3.11
            if [[ "$python_version" == "3.10" || "$python_version" == "3.11" ]]; then
                success "Python $python_version is compatible with AIOS"
            else
                error "Python $python_version is not compatible with AIOS (requires 3.10-3.11)"
                echo "  Resolution: AIOS environment should pin python = \">=3.10,<3.12\""
                ((ISSUES++))
            fi
            ;;
        *)
            # Default environments use 3.11-3.12
            if [[ "$python_version" == "3.11" || "$python_version" == "3.12" ]]; then
                success "Python $python_version is in supported range"
            else
                warn "Python $python_version may have compatibility issues"
                echo "  Recommendation: Use Python 3.11.x for best RoboStack compatibility"
            fi
            ;;
    esac
}

# Check PyTorch version coupling
check_pytorch_coupling() {
    local env="$1"
    info "Checking PyTorch version coupling for environment: $env"

    pixi run -e "$env" python << 'PYTHON' 2>/dev/null || return 0
import sys

# Version coupling rules
COUPLING = {
    '2.5': {'torchvision': '0.20', 'torchaudio': '2.5'},
    '2.4': {'torchvision': '0.19', 'torchaudio': '2.4'},
    '2.3': {'torchvision': '0.18', 'torchaudio': '2.3'},
    '2.2': {'torchvision': '0.17', 'torchaudio': '2.2'},
    '2.1': {'torchvision': '0.16', 'torchaudio': '2.1'},
}

try:
    import torch
    import torchvision
    import torchaudio
except ImportError as e:
    print(f"SKIP: PyTorch not fully installed ({e})")
    sys.exit(0)

torch_ver = '.'.join(torch.__version__.split('.')[:2])
tv_ver = '.'.join(torchvision.__version__.split('.')[:2])
ta_ver = '.'.join(torchaudio.__version__.split('.')[:2])

print(f"PyTorch: {torch.__version__} (major.minor: {torch_ver})")
print(f"TorchVision: {torchvision.__version__} (major.minor: {tv_ver})")
print(f"TorchAudio: {torchaudio.__version__} (major.minor: {ta_ver})")

if torch_ver not in COUPLING:
    print(f"WARN: No coupling rules for PyTorch {torch_ver}")
    sys.exit(0)

expected = COUPLING[torch_ver]
errors = []

if tv_ver != expected['torchvision']:
    errors.append(f"torchvision: expected {expected['torchvision']}.x, got {tv_ver}")

if ta_ver != expected['torchaudio']:
    errors.append(f"torchaudio: expected {expected['torchaudio']}.x, got {ta_ver}")

if errors:
    print("COUPLING_ERROR")
    for e in errors:
        print(f"  - {e}")
    print("\nResolution: Update pixi.toml with matching versions:")
    print(f"  pytorch = \">={torch_ver},<{float(torch_ver)+0.1}\"")
    print(f"  torchvision = \">={expected['torchvision']},<{float(expected['torchvision'])+0.01}\"")
    print(f"  torchaudio = \">={expected['torchaudio']},<{float(expected['torchaudio'])+0.1}\"")
    sys.exit(1)
else:
    print("OK: PyTorch ecosystem versions are correctly coupled")
PYTHON

    local result=$?
    if [[ $result -eq 1 ]]; then
        ((ISSUES++))
    fi
}

# Check for numpy compatibility
check_numpy_compat() {
    local env="$1"
    info "Checking NumPy compatibility for environment: $env"

    pixi run -e "$env" python << 'PYTHON' 2>/dev/null || return 0
import sys

try:
    import numpy as np
except ImportError:
    print("SKIP: NumPy not installed")
    sys.exit(0)

major_ver = int(np.__version__.split('.')[0])
print(f"NumPy version: {np.__version__}")

if major_ver >= 2:
    print("WARN: NumPy 2.x detected")
    print("  Some ROS2 and ML packages may not be compatible with NumPy 2.x")
    print("  Resolution: Pin numpy = \"<2\" in pixi.toml if you encounter issues")
    sys.exit(2)  # Warning, not error
else:
    print("OK: NumPy 1.x is widely compatible")
PYTHON

    local result=$?
    if [[ $result -eq 2 ]]; then
        warn "NumPy 2.x compatibility warning (non-blocking)"
    fi
}

# Check for pydantic version conflicts
check_pydantic() {
    local env="$1"
    info "Checking pydantic version for environment: $env"

    pixi run -e "$env" python << 'PYTHON' 2>/dev/null || return 0
import sys

try:
    import pydantic
except ImportError:
    print("SKIP: pydantic not installed")
    sys.exit(0)

version = pydantic.__version__
major = int(version.split('.')[0])

print(f"pydantic version: {version}")

# AIOS requires pydantic 2.7.0 exactly
if 'aios' in sys.argv[0] if len(sys.argv) > 0 else False:
    if version != "2.7.0":
        print("ERROR: AIOS requires pydantic==2.7.0")
        sys.exit(1)

if major == 1:
    print("WARN: pydantic v1 detected - consider upgrading to v2")
    print("  Resolution: Update to pydantic>=2.0 for better performance")
elif major == 2:
    print("OK: pydantic v2 in use")
PYTHON
}

# Check for common import conflicts
check_imports() {
    local env="$1"
    info "Testing critical package imports for environment: $env"

    pixi run -e "$env" python << 'PYTHON' 2>/dev/null
import sys
import warnings
warnings.filterwarnings('ignore')

critical_packages = [
    'numpy',
    'pandas',
    'torch',
    'transformers',
    'rclpy',  # ROS2
]

optional_packages = [
    'accelerate',
    'sentence_transformers',
    'mlflow',
    'chromadb',
]

print("Critical packages:")
critical_ok = True
for pkg in critical_packages:
    try:
        mod = __import__(pkg)
        version = getattr(mod, '__version__', 'unknown')
        print(f"  {pkg}: {version}")
    except ImportError:
        print(f"  {pkg}: NOT INSTALLED")
        critical_ok = False

print("\nOptional packages:")
for pkg in optional_packages:
    try:
        mod = __import__(pkg)
        version = getattr(mod, '__version__', 'unknown')
        print(f"  {pkg}: {version}")
    except ImportError:
        print(f"  {pkg}: not installed (optional)")

if not critical_ok:
    print("\nWARN: Some critical packages are missing")
    print("Resolution: Run 'pixi install' to install dependencies")
    sys.exit(2)
PYTHON
}

# Check a single environment
check_environment() {
    local env="$1"
    echo ""
    echo "========================================"
    echo "Environment: $env"
    echo "========================================"

    check_python_version "$env"
    check_pytorch_coupling "$env"
    check_numpy_compat "$env"
    check_pydantic "$env"
    check_imports "$env"
}

# Check all environments
check_all_environments() {
    local envs=("default" "cuda" "aios" "llmops" "finetuning" "caching")

    for env in "${envs[@]}"; do
        # Skip environments that might not be available
        if pixi run -e "$env" python --version &>/dev/null; then
            check_environment "$env"
        else
            warn "Environment '$env' not available or not installed"
        fi
    done
}

# Main
main() {
    echo "=========================================="
    echo "Python Dependency Conflict Checker"
    echo "=========================================="

    check_pixi

    if [[ "$ENV" == "--all" ]]; then
        check_all_environments
    else
        check_environment "$ENV"
    fi

    echo ""
    echo "=========================================="
    if [[ $ISSUES -eq 0 ]]; then
        success "No conflicts detected"
        exit 0
    else
        error "Found $ISSUES conflict(s) - see above for resolution steps"
        exit 1
    fi
}

# Help
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    echo "Usage: $0 [environment|--all]"
    echo ""
    echo "Check Python dependency conflicts in Pixi environments."
    echo ""
    echo "Environments:"
    echo "  default     Standard ROS2 + ML environment (default)"
    echo "  cuda        CUDA-enabled environment"
    echo "  aios        AIOS Agent OS environment"
    echo "  llmops      LLM evaluation environment"
    echo "  finetuning  Model training environment"
    echo "  caching     Prompt caching environment"
    echo ""
    echo "Options:"
    echo "  --all       Check all environments"
    echo "  -h, --help  Show this help"
    exit 0
fi

main
