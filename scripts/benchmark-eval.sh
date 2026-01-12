#!/usr/bin/env bash
# Benchmark Nix flake evaluation time
# Usage: ./scripts/benchmark-eval.sh
#
# This script measures the time to evaluate different flake outputs
# to help identify performance bottlenecks.
#
# Targets:
#   - flake show: < 10 seconds
#   - minimal shell: < 5 seconds
#   - default shell: < 15 seconds
#   - full shell: < 30 seconds

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}  Nix Flake Evaluation Benchmark${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""

# Function to benchmark a command
benchmark() {
    local name="$1"
    local target="$2"
    local cmd="$3"

    echo -e "${YELLOW}Benchmarking: ${name}${NC}"
    echo -e "  Target: ${target}s"

    # Run the command and capture time
    local start_time
    local end_time
    local duration

    start_time=$(date +%s.%N)
    if eval "$cmd" > /dev/null 2>&1; then
        end_time=$(date +%s.%N)
        duration=$(echo "$end_time - $start_time" | bc)

        # Check if within target
        local target_int
        target_int=$(echo "$target" | cut -d'.' -f1)
        local duration_int
        duration_int=$(echo "$duration" | cut -d'.' -f1)

        if [ "$duration_int" -le "$target_int" ]; then
            echo -e "  Result: ${GREEN}${duration}s${NC} (PASS)"
        else
            echo -e "  Result: ${RED}${duration}s${NC} (FAIL - exceeded target)"
        fi
    else
        echo -e "  Result: ${RED}FAILED${NC}"
    fi
    echo ""
}

# Check if nix is available
if ! command -v nix &> /dev/null; then
    echo -e "${RED}Error: nix command not found${NC}"
    echo "Please install Nix or enter a Nix environment first."
    exit 1
fi

# Get system info
echo -e "${BLUE}System Information:${NC}"
echo "  OS: $(uname -s)"
echo "  Arch: $(uname -m)"
echo "  Nix: $(nix --version)"
echo ""

# Run benchmarks
echo -e "${BLUE}Running Benchmarks:${NC}"
echo ""

# Flake show (metadata only)
benchmark "flake show" "10" "nix flake show --no-write-lock-file"

# Flake check (no build)
benchmark "flake check (no build)" "30" "nix flake check --no-build --no-write-lock-file"

# Minimal shell evaluation
benchmark "minimal shell eval" "5" "nix eval .#devShells.$(nix eval --impure --raw --expr 'builtins.currentSystem').minimal --apply 'x: x.name'"

# Default shell evaluation
benchmark "default shell eval" "15" "nix eval .#devShells.$(nix eval --impure --raw --expr 'builtins.currentSystem').default --apply 'x: x.name'"

# Full shell evaluation
benchmark "full shell eval" "30" "nix eval .#devShells.$(nix eval --impure --raw --expr 'builtins.currentSystem').full --apply 'x: x.name'"

# Summary
echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}  Benchmark Complete${NC}"
echo -e "${BLUE}======================================${NC}"
echo ""
echo "Tips for improving evaluation time:"
echo "  1. Use 'nix develop .#minimal' for CI/scripting"
echo "  2. Keep heavy shells (cuda, identity) off macOS"
echo "  3. Use binary caches (see flake.nix nixConfig)"
echo "  4. Profile with: nix eval --show-trace"
echo ""
