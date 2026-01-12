# Distributed systems packages - ruvector, QuDAG, and related tools
# BUILDKIT_STARTER_SPEC.md Layer 10: State & Storage
#
# RuVector: Distributed vector database with GNN self-learning
#   - npm install ruvector (Node.js bindings)
#   - cargo install ruvector-cli (CLI tools)
#   - https://github.com/ruvnet/ruvector
#
# QuDAG: Quantum-resistant distributed communication platform
#   - npm install qudag (Node.js bindings)
#   - cargo install qudag-cli (CLI tools)
#   - https://github.com/ruvnet/qudag
#
# Usage: packages.distributed
{ pkgs }:

let
  # RuVector source from git (for building from source if needed)
  ruvectorSrc = pkgs.fetchFromGitHub {
    owner = "ruvnet";
    repo = "ruvector";
    rev = "main";
    sha256 = "sha256-AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="; # Will need update
  };

  # QuDAG source from git (for building from source if needed)
  qudagSrc = pkgs.fetchFromGitHub {
    owner = "ruvnet";
    repo = "qudag";
    rev = "main";
    sha256 = "sha256-AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA="; # Will need update
  };

in
[
  # =================================================================
  # HTTP/gRPC Clients for RuVector and QuDAG
  # =================================================================
  pkgs.grpcurl           # gRPC CLI tool
  pkgs.grpcui            # gRPC web UI
  pkgs.protobuf          # Protocol buffers compiler
  pkgs.protoc-gen-go     # Go protobuf generator
  pkgs.protoc-gen-go-grpc # Go gRPC generator

  # =================================================================
  # Node.js packages for RuVector and QuDAG bindings
  # Install via npm: npm install ruvector qudag
  # =================================================================
  pkgs.nodejs_22         # Node.js LTS for npm packages
  pkgs.nodePackages.npm  # npm package manager

  # =================================================================
  # WebAssembly support for browser deployments
  # =================================================================
  pkgs.wasm-pack         # Rust to WASM compiler
  pkgs.wasm-bindgen-cli  # WASM bindings generator
  pkgs.binaryen          # WASM optimizer (wasm-opt)

  # =================================================================
  # Cryptography tools for QuDAG post-quantum verification
  # =================================================================
  pkgs.openssl           # TLS/SSL library
  pkgs.libsodium         # Modern crypto library
  pkgs.age               # Modern encryption tool
  pkgs.sops              # Secrets management

  # =================================================================
  # DAG and Graph tools
  # =================================================================
  pkgs.graphviz          # Graph visualization

  # =================================================================
  # WebSocket tools for QuDAG MCP connections
  # =================================================================
  pkgs.websocat          # WebSocket CLI client
]
