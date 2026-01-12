# LocalAI Models Directory

This directory stores AI models for LocalAI inference.

## Supported Model Formats

- GGUF (GPT-style models)
- GGML (legacy format)
- Safetensors

## Model Download

Use the provided script to download models:

```bash
./scripts/download-localai-models.sh
```

## Directory Structure

```
models/
├── README.md           # This file
├── llm/               # Large language models
├── embedding/         # Embedding models
└── vision/            # Vision models
```

## Configuration

Models are automatically discovered by LocalAI when placed in this directory.
See `manifests/kubernetes/base/inference/localai.yaml` for deployment configuration.

## Resources

- [LocalAI Documentation](https://localai.io/docs/)
- [GGUF Models on HuggingFace](https://huggingface.co/models?library=gguf)
