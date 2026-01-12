#!/usr/bin/env bash
# =============================================================================
# Argo Rollouts Installation Script
# =============================================================================
# P1-006: Install Argo Rollouts for progressive delivery
#
# Usage:
#   ./scripts/install-argo-rollouts.sh           # Install Argo Rollouts
#   ./scripts/install-argo-rollouts.sh --uninstall  # Uninstall
#   ./scripts/install-argo-rollouts.sh --status     # Check status
#
# Requirements:
#   - kubectl
#   - Access to Kubernetes cluster
# =============================================================================

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MANIFESTS_DIR="${PROJECT_ROOT}/manifests/kubernetes/base/delivery"
NAMESPACE="argo-rollouts"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() { echo -e "${BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${GREEN}[OK]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# Check prerequisites
check_prereqs() {
    log_info "Checking prerequisites..."

    if ! command -v kubectl &>/dev/null; then
        log_error "kubectl not found. Please install kubectl."
        exit 1
    fi

    if ! kubectl cluster-info &>/dev/null; then
        log_error "Cannot connect to Kubernetes cluster. Please check your kubeconfig."
        exit 1
    fi

    log_success "Prerequisites satisfied"
}

# Install Argo Rollouts
install() {
    log_info "Installing Argo Rollouts..."

    # Apply manifests
    kubectl apply -k "${MANIFESTS_DIR}"

    log_success "Argo Rollouts manifests applied"

    # Wait for deployment
    log_info "Waiting for Argo Rollouts to be ready..."
    kubectl wait --for=condition=available --timeout=300s \
        deployment/argo-rollouts -n "${NAMESPACE}" || {
        log_error "Timeout waiting for Argo Rollouts deployment"
        exit 1
    }

    log_success "Argo Rollouts is ready"

    # Print dashboard access instructions
    echo ""
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           Argo Rollouts Installed Successfully                 ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo ""
    echo "Access the Argo Rollouts dashboard:"
    echo "  kubectl port-forward -n ${NAMESPACE} svc/argo-rollouts-dashboard 3100:3100"
    echo "  Open: http://localhost:3100"
    echo ""
    echo "Install kubectl plugin:"
    echo "  curl -LO https://github.com/argoproj/argo-rollouts/releases/latest/download/kubectl-argo-rollouts-linux-amd64"
    echo "  chmod +x kubectl-argo-rollouts-linux-amd64"
    echo "  sudo mv kubectl-argo-rollouts-linux-amd64 /usr/local/bin/kubectl-argo-rollouts"
    echo ""
    echo "Example usage:"
    echo "  kubectl argo rollouts list rollouts -A"
    echo "  kubectl argo rollouts get rollout <rollout-name> --watch"
    echo ""
}

# Uninstall Argo Rollouts
uninstall() {
    log_warn "Uninstalling Argo Rollouts..."

    kubectl delete -k "${MANIFESTS_DIR}" || {
        log_warn "Some resources may not exist"
    }

    log_success "Argo Rollouts uninstalled"
}

# Check status
status() {
    log_info "Checking Argo Rollouts status..."
    echo ""

    if kubectl get namespace "${NAMESPACE}" &>/dev/null; then
        log_success "Namespace: ${NAMESPACE} exists"
    else
        log_error "Namespace: ${NAMESPACE} not found"
        exit 1
    fi

    echo ""
    log_info "Deployments:"
    kubectl get deployments -n "${NAMESPACE}" -o wide

    echo ""
    log_info "Pods:"
    kubectl get pods -n "${NAMESPACE}" -o wide

    echo ""
    log_info "Services:"
    kubectl get services -n "${NAMESPACE}" -o wide

    echo ""
    # Check if rollouts CRD exists
    if kubectl get crd rollouts.argoproj.io &>/dev/null; then
        log_success "Rollouts CRD is installed"
        echo ""
        log_info "Active Rollouts across all namespaces:"
        kubectl get rollouts -A || log_warn "No rollouts found"
    else
        log_warn "Rollouts CRD not found. Install with: kubectl create -f https://github.com/argoproj/argo-rollouts/releases/latest/download/install.yaml"
    fi
}

# Print usage
usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Install and manage Argo Rollouts for progressive delivery.

Options:
    --install     Install Argo Rollouts (default)
    --uninstall   Uninstall Argo Rollouts
    --status      Check Argo Rollouts status
    --help        Show this help message

Examples:
    $(basename "$0")                    # Install
    $(basename "$0") --status           # Check status
    $(basename "$0") --uninstall        # Uninstall
EOF
}

# Main
main() {
    local action="install"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --install)
                action="install"
                shift
                ;;
            --uninstall)
                action="uninstall"
                shift
                ;;
            --status)
                action="status"
                shift
                ;;
            --help|-h)
                usage
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                usage
                exit 1
                ;;
        esac
    done

    echo ""
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           Argo Rollouts Installation Script                    ║"
    echo "║           P1-006: Progressive Delivery                          ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo ""

    check_prereqs

    case $action in
        install)
            install
            ;;
        uninstall)
            uninstall
            ;;
        status)
            status
            ;;
    esac
}

main "$@"
