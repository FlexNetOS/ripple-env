#!/bin/bash
set -e

echo "Waiting for k3s to be ready..."
until kubectl get nodes 2>/dev/null; do 
  echo "  Still waiting..."
  sleep 5
done

echo "Installing Argo CD..."
kubectl create namespace argocd || true
kubectl apply -n argocd -f https://raw.githubusercontent.com/argoproj/argo-cd/stable/manifests/install.yaml

echo "Installing Argo Rollouts..."
kubectl create namespace argo-rollouts || true
kubectl apply -n argo-rollouts -f https://github.com/argoproj/argo-rollouts/releases/latest/download/install.yaml

echo "Installing Argo Workflows..."
kubectl create namespace argo || true
kubectl apply -n argo -f https://github.com/argoproj/argo-workflows/releases/latest/download/install.yaml

echo "Waiting for Argo CD to be ready..."
kubectl wait --for=condition=available --timeout=300s deployment/argocd-server -n argocd

echo "Patching Argo CD service to NodePort..."
kubectl patch svc argocd-server -n argocd -p '{"spec": {"type": "NodePort", "ports": [{"port": 443, "nodePort": 30443, "targetPort": 8080}]}}'

echo ""
echo "=== Installation complete! ==="
echo ""
echo "Initial admin password:"
kubectl -n argocd get secret argocd-initial-admin-secret -o jsonpath="{.data.password}" | base64 -d
echo ""
echo ""
echo "Access Argo CD UI at: https://localhost:30443"
echo "Username: admin"
echo ""
