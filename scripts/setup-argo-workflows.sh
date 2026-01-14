#!/bin/bash
# Setup script for Argo Workflows
# Part of ARIA infrastructure - P1-011 implementation

set -e

echo "========================================"
echo "Argo Workflows Setup Script"
echo "========================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if kubeconfig is set
if [ -z "$KUBECONFIG" ]; then
    export KUBECONFIG=./data/k3s/kubeconfig/kubeconfig.yaml
    echo -e "${YELLOW}KUBECONFIG not set, using: $KUBECONFIG${NC}"
fi

# Function to check if kubectl is working
check_kubectl() {
    echo "Checking kubectl connectivity..."
    if ! kubectl get nodes > /dev/null 2>&1; then
        echo -e "${RED}Error: Cannot connect to Kubernetes cluster${NC}"
        echo "Make sure k3s is running: docker compose -f docker/docker-compose.argo.yml up -d"
        exit 1
    fi
    echo -e "${GREEN}✓ kubectl connectivity verified${NC}"
}

# Function to check if Argo is installed
check_argo_installed() {
    echo ""
    echo "Checking Argo Workflows installation..."
    if ! kubectl get namespace argo > /dev/null 2>&1; then
        echo -e "${RED}Error: Argo namespace not found${NC}"
        echo "Run the installation first: docker compose -f docker/docker-compose.argo.yml up -d"
        exit 1
    fi
    echo -e "${GREEN}✓ Argo namespace exists${NC}"
}

# Function to apply configurations
apply_configurations() {
    echo ""
    echo "Applying Argo Workflows configurations..."

    # Apply RBAC
    echo "  - Applying RBAC configuration..."
    kubectl apply -f manifests/argo-workflows/config/rbac.yaml

    # Apply workflow controller config
    echo "  - Applying workflow controller configuration..."
    kubectl apply -f manifests/argo-workflows/config/workflow-controller-configmap.yaml

    # Apply service configuration
    echo "  - Configuring Argo Server service..."
    kubectl patch svc argo-server -n argo -p '{"spec": {"type": "NodePort", "ports": [{"name": "web", "port": 2746, "targetPort": 2746, "nodePort": 32746, "protocol": "TCP"}]}}' || true

    echo -e "${GREEN}✓ Configurations applied${NC}"
}

# Function to install workflow templates
install_templates() {
    echo ""
    echo "Installing workflow templates..."

    templates=(
        "manifests/argo-workflows/templates/ci-pipeline.yaml"
        "manifests/argo-workflows/templates/ml-pipeline.yaml"
        "manifests/argo-workflows/templates/parallel-processing.yaml"
        "manifests/argo-workflows/templates/conditional-workflow.yaml"
    )

    for template in "${templates[@]}"; do
        template_name=$(basename "$template" .yaml)
        echo "  - Installing $template_name..."
        kubectl apply -f "$template"
    done

    echo -e "${GREEN}✓ Workflow templates installed${NC}"
}

# Function to restart controllers
restart_controllers() {
    echo ""
    echo "Restarting Argo controllers to apply changes..."
    kubectl rollout restart deployment/workflow-controller -n argo
    kubectl rollout restart deployment/argo-server -n argo

    echo "Waiting for rollout to complete..."
    kubectl rollout status deployment/workflow-controller -n argo --timeout=120s
    kubectl rollout status deployment/argo-server -n argo --timeout=120s

    echo -e "${GREEN}✓ Controllers restarted${NC}"
}

# Function to verify installation
verify_installation() {
    echo ""
    echo "Verifying Argo Workflows installation..."

    # Check pods
    echo "  - Checking pods..."
    kubectl get pods -n argo

    # Check services
    echo ""
    echo "  - Checking services..."
    kubectl get svc -n argo

    # Check workflow templates
    echo ""
    echo "  - Checking workflow templates..."
    kubectl get workflowtemplate -n argo

    echo ""
    echo -e "${GREEN}✓ Verification complete${NC}"
}

# Function to run test workflow
run_test_workflow() {
    echo ""
    read -p "Do you want to run a test workflow? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Submitting hello-world test workflow..."
        kubectl create -f manifests/argo-workflows/templates/hello-world.yaml -n argo

        echo ""
        echo "Waiting for workflow to start..."
        sleep 5

        echo ""
        echo "Workflow status:"
        kubectl get workflows -n argo

        echo ""
        echo -e "${YELLOW}To watch the workflow:${NC}"
        echo "  kubectl get workflows -n argo --watch"
        echo ""
        echo -e "${YELLOW}To view logs:${NC}"
        echo "  kubectl logs -n argo -l workflows.argoproj.io/workflow --tail=-1"
    fi
}

# Function to display access information
display_access_info() {
    echo ""
    echo "========================================"
    echo "Argo Workflows Setup Complete!"
    echo "========================================"
    echo ""
    echo -e "${GREEN}Access Information:${NC}"
    echo ""
    echo "Argo Workflows UI:"
    echo "  - Direct access: http://localhost:2746"
    echo "  - NodePort access: http://localhost:32746"
    echo ""
    echo "Port forwarding (alternative):"
    echo "  kubectl -n argo port-forward svc/argo-server 2746:2746"
    echo ""
    echo "Common Commands:"
    echo "  - List workflows:        kubectl get workflows -n argo"
    echo "  - Get workflow details:  kubectl get workflow <name> -n argo -o yaml"
    echo "  - View logs:            kubectl logs -n argo <pod-name>"
    echo "  - List templates:       kubectl get workflowtemplate -n argo"
    echo ""
    echo "Argo CLI (if installed):"
    echo "  - List workflows:        argo list -n argo"
    echo "  - Watch workflow:        argo watch <workflow-name> -n argo"
    echo "  - Get logs:             argo logs <workflow-name> -n argo"
    echo ""
    echo "Documentation:"
    echo "  - See manifests/argo-workflows/README.md for detailed usage"
    echo "  - Official docs: https://argoproj.github.io/argo-workflows/"
    echo ""
}

# Main execution
main() {
    check_kubectl
    check_argo_installed
    apply_configurations
    install_templates
    restart_controllers
    verify_installation
    run_test_workflow
    display_access_info
}

# Run main function
main
