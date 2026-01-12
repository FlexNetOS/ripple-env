# LocalAI Model Management Guide

**P1-008: LocalAI Model Download & Configuration**

This guide covers downloading, configuring, and managing AI models for LocalAI in the agentic infrastructure.

## Table of Contents

- [Overview](#overview)
- [Quick Start](#quick-start)
- [Model Types](#model-types)
- [Downloading Models](#downloading-models)
- [Model Configuration](#model-configuration)
- [Performance Tuning](#performance-tuning)
- [Troubleshooting](#troubleshooting)

## Overview

LocalAI supports multiple model formats and backends:

- **GGUF/GGML**: Quantized models for efficient CPU/GPU inference
- **PyTorch**: Full-precision models with transformers
- **ONNX**: Cross-platform optimized models
- **TensorFlow**: TF SavedModel format

## Quick Start

### 1. Using the Download Script

The repository includes a model download script:

```bash
# Download a specific model
./scripts/download-models.sh --model mistral-7b-instruct

# Download multiple models
./scripts/download-models.sh --model llama2-7b --model codellama-13b

# List available models
./scripts/download-models.sh --list

# Download to custom directory
./scripts/download-models.sh --model phi-2 --output /data/models
```

### 2. Manual Download

Download models directly from Hugging Face:

```bash
# Create models directory
mkdir -p /home/user/ripple-env/models

# Download using huggingface-cli
huggingface-cli download \
    TheBloke/Mistral-7B-Instruct-v0.2-GGUF \
    mistral-7b-instruct-v0.2.Q4_K_M.gguf \
    --local-dir /home/user/ripple-env/models \
    --local-dir-use-symlinks False
```

### 3. Using Docker Volume

Mount models directory when running LocalAI:

```bash
docker run -d \
    --name localai \
    -p 8080:8080 \
    -v /home/user/ripple-env/models:/models:ro \
    -v /home/user/ripple-env/config/localai:/config:ro \
    --env MODELS_PATH=/models \
    localai/localai:latest
```

## Model Types

### Text Generation Models

#### Mistral 7B Instruct (Recommended)
- **Size**: 4.4GB (Q4_K_M quantization)
- **Use Case**: General-purpose instruction following
- **Download**:
  ```bash
  huggingface-cli download TheBloke/Mistral-7B-Instruct-v0.2-GGUF \
      mistral-7b-instruct-v0.2.Q4_K_M.gguf \
      --local-dir models/mistral-7b
  ```

#### Llama 2 7B Chat
- **Size**: 3.8GB (Q4_0 quantization)
- **Use Case**: Conversational AI, chat applications
- **Download**:
  ```bash
  huggingface-cli download TheBloke/Llama-2-7B-Chat-GGUF \
      llama-2-7b-chat.Q4_0.gguf \
      --local-dir models/llama2-7b
  ```

#### Phi-2 (2.7B)
- **Size**: 1.6GB (Q4_K_M quantization)
- **Use Case**: Lightweight reasoning and coding
- **Download**:
  ```bash
  huggingface-cli download TheBloke/phi-2-GGUF \
      phi-2.Q4_K_M.gguf \
      --local-dir models/phi-2
  ```

### Code Generation Models

#### CodeLlama 13B Instruct
- **Size**: 7.4GB (Q4_K_M quantization)
- **Use Case**: Code generation and completion
- **Download**:
  ```bash
  huggingface-cli download TheBloke/CodeLlama-13B-Instruct-GGUF \
      codellama-13b-instruct.Q4_K_M.gguf \
      --local-dir models/codellama-13b
  ```

#### Deepseek Coder 6.7B
- **Size**: 3.9GB (Q4_K_M quantization)
- **Use Case**: Code completion and infilling
- **Download**:
  ```bash
  huggingface-cli download TheBloke/deepseek-coder-6.7B-instruct-GGUF \
      deepseek-coder-6.7b-instruct.Q4_K_M.gguf \
      --local-dir models/deepseek-coder
  ```

### Embedding Models

#### all-MiniLM-L6-v2
- **Size**: 90MB
- **Use Case**: Text embeddings for RAG, search
- **Download**:
  ```bash
  huggingface-cli download sentence-transformers/all-MiniLM-L6-v2 \
      --local-dir models/all-MiniLM-L6-v2
  ```

#### BGE-Base-EN-v1.5
- **Size**: 440MB
- **Use Case**: High-quality English embeddings
- **Download**:
  ```bash
  huggingface-cli download BAAI/bge-base-en-v1.5 \
      --local-dir models/bge-base-en-v1.5
  ```

## Downloading Models

### Method 1: Hugging Face CLI (Recommended)

Install the Hugging Face CLI:

```bash
pip install huggingface-hub
```

Download a model:

```bash
# Login (optional, for gated models)
huggingface-cli login

# Download model
huggingface-cli download \
    <repo-id> \
    <filename> \
    --local-dir <output-dir> \
    --local-dir-use-symlinks False
```

### Method 2: wget/curl

Download directly via URL:

```bash
# Example: Download from Hugging Face CDN
wget https://huggingface.co/TheBloke/Mistral-7B-Instruct-v0.2-GGUF/resolve/main/mistral-7b-instruct-v0.2.Q4_K_M.gguf \
    -O models/mistral-7b-instruct-v0.2.Q4_K_M.gguf
```

### Method 3: Using the Download Script

The repository includes `scripts/download-models.sh` with predefined model configurations:

```bash
# Show available models
./scripts/download-models.sh --list

# Download specific model
./scripts/download-models.sh --model mistral-7b-instruct

# Download with progress bar
./scripts/download-models.sh --model llama2-7b --verbose
```

## Model Configuration

### Creating Model Config Files

LocalAI uses YAML configuration files for each model:

```yaml
# config/localai/models/mistral-7b.yaml
name: mistral-7b-instruct
backend: llama
parameters:
  model: mistral-7b-instruct-v0.2.Q4_K_M.gguf
  context_size: 4096
  threads: 8
  gpu_layers: 0  # Set to 35+ for GPU acceleration
  temperature: 0.7
  top_k: 40
  top_p: 0.9
  repeat_penalty: 1.1
  batch_size: 512
  f16: true
template:
  chat: |
    <s>[INST] {{.Input}} [/INST]
  completion: |
    {{.Input}}
```

### Model Directory Structure

Organize models and configs:

```
models/
├── mistral-7b/
│   └── mistral-7b-instruct-v0.2.Q4_K_M.gguf
├── llama2-7b/
│   └── llama-2-7b-chat.Q4_0.gguf
└── embeddings/
    └── all-MiniLM-L6-v2/

config/localai/models/
├── mistral-7b.yaml
├── llama2-7b.yaml
└── embeddings.yaml
```

## Performance Tuning

### CPU Optimization

For CPU-only inference:

```yaml
parameters:
  threads: 8           # Match your CPU cores
  batch_size: 512      # Increase for throughput
  context_size: 2048   # Reduce for faster processing
  gpu_layers: 0        # Disable GPU
  mlock: true          # Keep model in RAM
  mmap: true           # Memory-map model file
```

### GPU Acceleration

For NVIDIA GPU (CUDA):

```yaml
parameters:
  gpu_layers: 35       # Number of layers to offload to GPU
  threads: 4           # Fewer threads when using GPU
  batch_size: 2048     # Larger batch for GPU
  context_size: 4096   # Full context with GPU
  tensor_split: "0,1"  # Multi-GPU configuration
```

### Memory Management

For systems with limited RAM:

```yaml
parameters:
  context_size: 2048   # Reduce context window
  batch_size: 256      # Smaller batch size
  low_vram: true       # Enable low VRAM mode
  numa: true           # NUMA-aware allocation
  use_mlock: false     # Don't lock in RAM
```

## Model Quantization Levels

GGUF models come in different quantization levels:

| Quantization | Size Reduction | Quality | Use Case |
|--------------|----------------|---------|----------|
| Q2_K         | ~75%          | Low     | Extreme compression |
| Q3_K_M       | ~60%          | Medium  | Balanced mobile/edge |
| Q4_0         | ~50%          | Good    | General purpose |
| Q4_K_M       | ~50%          | Better  | **Recommended** |
| Q5_K_M       | ~40%          | High    | Better quality |
| Q6_K         | ~30%          | Very High | Maximum quality |
| Q8_0         | ~15%          | Excellent | Near-original quality |

**Recommendation**: Start with Q4_K_M for best quality/size tradeoff.

## Troubleshooting

### Model Not Loading

Check logs:
```bash
docker logs localai
```

Common issues:
- **File not found**: Verify model path in config
- **Out of memory**: Reduce context_size or batch_size
- **Incompatible format**: Ensure model format matches backend

### Slow Inference

Optimization checklist:
- [ ] Use appropriate quantization level (Q4_K_M)
- [ ] Increase thread count to match CPU cores
- [ ] Enable GPU layers if GPU available
- [ ] Reduce context_size if not needed
- [ ] Enable mmap and mlock for faster loading

### GPU Not Detected

For CUDA GPUs:
```bash
# Check GPU availability
nvidia-smi

# Rebuild LocalAI with CUDA support
docker run -d \
    --name localai \
    --gpus all \
    -p 8080:8080 \
    -v $(pwd)/models:/models \
    localai/localai:latest-gpu-cuda-12
```

## Recommended Model Combinations

### Lightweight Setup (< 8GB RAM)
- **Text**: Phi-2 (1.6GB)
- **Embeddings**: all-MiniLM-L6-v2 (90MB)
- **Total**: ~2GB

### Balanced Setup (16GB RAM)
- **Text**: Mistral 7B Q4_K_M (4.4GB)
- **Code**: Deepseek Coder 6.7B (3.9GB)
- **Embeddings**: BGE-Base-EN-v1.5 (440MB)
- **Total**: ~9GB

### High-Performance Setup (32GB+ RAM)
- **Text**: Mistral 7B Q6_K (6.2GB)
- **Code**: CodeLlama 13B Q5_K_M (9.1GB)
- **Reasoning**: Llama 2 13B Q5_K_M (9.1GB)
- **Embeddings**: BGE-Large-EN-v1.5 (1.3GB)
- **Total**: ~26GB

## Integration with Docker Compose

Add models to your docker-compose configuration:

```yaml
services:
  localai:
    image: localai/localai:latest
    volumes:
      - ./models:/models:ro
      - ./config/localai:/config:ro
    environment:
      - MODELS_PATH=/models
      - CONFIG_PATH=/config
      - THREADS=8
      - CONTEXT_SIZE=4096
```

## Additional Resources

- [LocalAI Documentation](https://localai.io/docs/)
- [Hugging Face Model Hub](https://huggingface.co/models)
- [GGUF Model Zoo](https://huggingface.co/TheBloke)
- [Model Quantization Guide](https://github.com/ggerganov/llama.cpp/blob/master/examples/quantize/README.md)

## Model Licenses

Always check model licenses before use:
- **Llama 2**: Meta Community License
- **Mistral**: Apache 2.0
- **Phi-2**: MIT License
- **CodeLlama**: Meta Community License
- **Deepseek Coder**: Deepseek License

For commercial use, verify the license permits your use case.
