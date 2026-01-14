#!/bin/bash
# Verification script for Argo Workflows
# Part of ARIA infrastructure - P1-011 implementation

set -euo pipefail

echo "========================================"
echo "Argo Workflows Verification Script"
echo "========================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Counters
PASSED=0
FAILED=0
WARNINGS=0

# Prefer v2 plugin (`docker compose`), fallback to legacy `docker-compose`.
COMPOSE=("docker" "compose")
if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
    if command -v docker-compose >/dev/null 2>&1; then
        COMPOSE=("docker-compose")
    else
        COMPOSE=()
    fi
fi

ARGO_COMPOSE_FILE="docker-compose.argo.yml"
if [ -f "docker/docker-compose.argo.yml" ]; then
    ARGO_COMPOSE_FILE="docker/docker-compose.argo.yml"
fi

ARGO_REQUIRE_CLUSTER="${ARGO_REQUIRE_CLUSTER:-0}"

# Check if kubeconfig is set
if [ -z "${KUBECONFIG:-}" ]; then
    export KUBECONFIG=./data/k3s/kubeconfig/kubeconfig.yaml
    echo -e "${YELLOW}KUBECONFIG not set, using: $KUBECONFIG${NC}"
fi

# Function to print test result
print_test() {
    local status=$1
    local message=$2
    if [ "$status" == "PASS" ]; then
        echo -e "${GREEN}✓${NC} $message"
        PASSED=$((PASSED + 1))
    elif [ "$status" == "FAIL" ]; then
        echo -e "${RED}✗${NC} $message"
        FAILED=$((FAILED + 1))
    elif [ "$status" == "WARN" ]; then
        echo -e "${YELLOW}⚠${NC} $message"
        WARNINGS=$((WARNINGS + 1))
    else
        echo -e "${BLUE}ℹ${NC} $message"
    fi
}

# Function to print section header
print_section() {
    echo ""
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

# 1. Check kubectl connectivity
print_section "1. Kubernetes Connectivity"
if kubectl get nodes > /dev/null 2>&1; then
    print_test "PASS" "kubectl can connect to cluster"
    kubectl get nodes
else
    if [ "$ARGO_REQUIRE_CLUSTER" = "1" ]; then
        print_test "FAIL" "Cannot connect to Kubernetes cluster"
        echo -e "${RED}Please start k3s:${NC} ${COMPOSE[*]:-docker compose} -f $ARGO_COMPOSE_FILE up -d"
        exit 1
    fi

    print_test "WARN" "Cannot connect to Kubernetes cluster (skipping live checks)"
    echo -e "${YELLOW}To run this verifier against a live cluster:${NC}"
    echo -e "  1) Start k3s: ${COMPOSE[*]:-docker compose} -f $ARGO_COMPOSE_FILE up -d"
    echo -e "  2) Or set ARGO_REQUIRE_CLUSTER=1 to fail when the cluster is unreachable"
    exit 0
fi

# 2. Check namespaces
print_section "2. Namespace Verification"
if kubectl get namespace argo > /dev/null 2>&1; then
    print_test "PASS" "Argo namespace exists"
else
    print_test "FAIL" "Argo namespace not found"
fi

if kubectl get namespace argocd > /dev/null 2>&1; then
    print_test "PASS" "ArgoCD namespace exists"
else
    print_test "WARN" "ArgoCD namespace not found"
fi

if kubectl get namespace argo-rollouts > /dev/null 2>&1; then
    print_test "PASS" "Argo Rollouts namespace exists"
else
    print_test "WARN" "Argo Rollouts namespace not found"
fi

# 3. Check Argo Workflows deployments
print_section "3. Argo Workflows Deployments"
if kubectl get deployment workflow-controller -n argo > /dev/null 2>&1; then
    status=$(kubectl get deployment workflow-controller -n argo -o jsonpath='{.status.conditions[?(@.type=="Available")].status}')
    if [ "$status" == "True" ]; then
        print_test "PASS" "workflow-controller deployment is available"
    else
        print_test "FAIL" "workflow-controller deployment is not available"
    fi
else
    print_test "FAIL" "workflow-controller deployment not found"
fi

if kubectl get deployment argo-server -n argo > /dev/null 2>&1; then
    status=$(kubectl get deployment argo-server -n argo -o jsonpath='{.status.conditions[?(@.type=="Available")].status}')
    if [ "$status" == "True" ]; then
        print_test "PASS" "argo-server deployment is available"
    else
        print_test "FAIL" "argo-server deployment is not available"
    fi
else
    print_test "FAIL" "argo-server deployment not found"
fi

# 4. Check pods
print_section "4. Pod Status"
echo ""
kubectl get pods -n argo
echo ""

running_pods=$(kubectl get pods -n argo --field-selector=status.phase=Running --no-headers 2>/dev/null | wc -l)
if [ "$running_pods" -ge 2 ]; then
    print_test "PASS" "$running_pods pods running in argo namespace"
else
    print_test "FAIL" "Only $running_pods pods running (expected at least 2)"
fi

# 5. Check services
print_section "5. Service Configuration"
if kubectl get svc argo-server -n argo > /dev/null 2>&1; then
    print_test "PASS" "argo-server service exists"
    svc_type=$(kubectl get svc argo-server -n argo -o jsonpath='{.spec.type}')
    if [ "$svc_type" == "NodePort" ] || [ "$svc_type" == "LoadBalancer" ]; then
        print_test "PASS" "argo-server service type: $svc_type"
    else
        print_test "WARN" "argo-server service type: $svc_type (consider NodePort for easier access)"
    fi
else
    print_test "FAIL" "argo-server service not found"
fi

# 6. Check service accounts
print_section "6. RBAC Configuration"
if kubectl get serviceaccount argo-workflow -n argo > /dev/null 2>&1; then
    print_test "PASS" "argo-workflow service account exists"
else
    print_test "WARN" "argo-workflow service account not found (run setup-argo-workflows.sh)"
fi

if kubectl get role argo-workflow-role -n argo > /dev/null 2>&1; then
    print_test "PASS" "argo-workflow-role exists"
else
    print_test "WARN" "argo-workflow-role not found (run setup-argo-workflows.sh)"
fi

if kubectl get rolebinding argo-workflow-binding -n argo > /dev/null 2>&1; then
    print_test "PASS" "argo-workflow-binding exists"
else
    print_test "WARN" "argo-workflow-binding not found (run setup-argo-workflows.sh)"
fi

# 7. Check ConfigMap
print_section "7. Configuration"
if kubectl get configmap workflow-controller-configmap -n argo > /dev/null 2>&1; then
    print_test "PASS" "workflow-controller-configmap exists"
else
    print_test "WARN" "workflow-controller-configmap not found (using defaults)"
fi

# 8. Check WorkflowTemplates
print_section "8. Workflow Templates"
template_count=$(kubectl get workflowtemplate -n argo --no-headers 2>/dev/null | wc -l)
if [ "$template_count" -gt 0 ]; then
    print_test "PASS" "$template_count workflow template(s) installed"
    echo ""
    kubectl get workflowtemplate -n argo
else
    print_test "WARN" "No workflow templates found (run setup-argo-workflows.sh)"
fi

# 9. Check if workflows can be created
print_section "9. Workflow Execution Test"
echo "Checking if workflows can be submitted..."
if kubectl auth can-i create workflows.argoproj.io -n argo > /dev/null 2>&1; then
    print_test "PASS" "Can create workflows in argo namespace"
else
    print_test "FAIL" "Cannot create workflows (check RBAC permissions)"
fi

# 10. Check port accessibility
print_section "10. Network Access"
if netstat -tuln 2>/dev/null | grep -q ":2746 " || ss -tuln 2>/dev/null | grep -q ":2746 "; then
    print_test "PASS" "Port 2746 is listening (Argo Workflows UI)"
else
    print_test "WARN" "Port 2746 not accessible (may need port forwarding)"
fi

# 11. Check recent workflows
print_section "11. Workflow History"
workflow_count=$(kubectl get workflows -n argo --no-headers 2>/dev/null | wc -l)
if [ "$workflow_count" -gt 0 ]; then
    print_test "INFO" "$workflow_count workflow(s) found"
    echo ""
    kubectl get workflows -n argo
else
    print_test "INFO" "No workflows found (this is normal for new installation)"
fi

# 12. Component versions
print_section "12. Component Versions"
echo ""
echo "Workflow Controller:"
kubectl get deployment workflow-controller -n argo -o jsonpath='{.spec.template.spec.containers[0].image}'
echo ""
echo ""
echo "Argo Server:"
kubectl get deployment argo-server -n argo -o jsonpath='{.spec.template.spec.containers[0].image}'
echo ""

# Summary
print_section "Verification Summary"
echo ""
echo -e "Tests Passed:    ${GREEN}$PASSED${NC}"
echo -e "Tests Failed:    ${RED}$FAILED${NC}"
echo -e "Warnings:        ${YELLOW}$WARNINGS${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ Argo Workflows verification completed successfully!${NC}"
    echo ""
    echo -e "${BLUE}Access Argo Workflows UI:${NC}"
    echo "  - Direct: http://localhost:2746"
    echo "  - Port forward: kubectl -n argo port-forward svc/argo-server 2746:2746"
    echo ""
    echo -e "${BLUE}Next steps:${NC}"
    if [ $WARNINGS -gt 0 ]; then
        echo "  1. Run ./scripts/setup-argo-workflows.sh to complete configuration"
        echo "  2. Submit test workflow: kubectl create -f manifests/argo-workflows/templates/hello-world.yaml -n argo"
        echo "  3. View workflow: kubectl get workflows -n argo"
    else
        echo "  1. Submit test workflow: kubectl create -f manifests/argo-workflows/templates/hello-world.yaml -n argo"
        echo "  2. View workflow: kubectl get workflows -n argo"
        echo "  3. Access UI: http://localhost:2746"
    fi
    echo ""
    echo -e "${BLUE}Documentation:${NC}"
    echo "  See manifests/argo-workflows/README.md for detailed usage"
    echo ""
    exit 0
else
    echo -e "${RED}✗ Argo Workflows verification failed!${NC}"
    echo ""
    echo -e "${YELLOW}Troubleshooting steps:${NC}"
    echo "  1. Check if k3s is running: docker ps | grep k3s"
    echo "  2. View installer logs: ${COMPOSE[*]:-docker compose} -f $ARGO_COMPOSE_FILE logs argocd-installer"
    echo "  3. Check pod logs: kubectl logs -n argo -l app=workflow-controller"
    echo "  4. Check argo server logs: kubectl logs -n argo -l app=argo-server"
    echo "  5. Restart installation: ${COMPOSE[*]:-docker compose} -f $ARGO_COMPOSE_FILE down && ${COMPOSE[*]:-docker compose} -f $ARGO_COMPOSE_FILE up -d"
    echo ""
    exit 1
fi
