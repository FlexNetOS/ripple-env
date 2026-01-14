# Script Contract: download-models.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/download-models.sh`

---

## Purpose

LocalAI GGUF model downloader for MOE (Multi-Model Fan-Out) inference. Downloads 7 models from HuggingFace (5 required, 2 optional) totaling ~15GB for min_parallel requirement. Supports verification, listing, and cleanup. Models defined in `manifests/distributed/inference_policy.yaml`.

---

## Invocation

```bash
./scripts/download-models.sh [OPTIONS]
```

**Options:**
- (no options) - Download required models only
- `--all` - Include optional models (~20GB)
- `--list` - List models without downloading
- `--verify` - Verify existing downloads
- `--clean` - Remove all models
- `--help` - Show help

---

## Side Effects

### Model Files (lines 39-48, 216)
Downloads to `data/localai/models/`:
- gemma-3n-E2B-it-UD-Q4_K_XL.gguf (2.1GB)
- Phi-4-mini-reasoning-UD-Q4_K_XL.gguf (2.5GB)
- DeepSeek-R1-Distill-Qwen-1.5B-Q8_0.gguf (1.6GB)
- gemma-3-4b-it-qat-UD-Q4_K_XL.gguf (2.8GB)
- Qwen3-4B-Q4_K_M.gguf (2.7GB)

---

## Safety Classification

**🟢 SAFE** - Downloads files to data directory.

---

## Key Features

### Model Definitions (lines 39-49)
Format: `name|filename|repo|size_bytes|required`

### Download Resume (lines 239-245)
Skips existing files with size validation (±10% tolerance).

### Verification (lines 152-209)
Checks MOE minimum requirement: 5+ models.

---

## References

- **Main script:** `scripts/download-models.sh` (358 lines)
- **Model list:** lines 39-49
- **Download logic:** lines 212-280
- **Policy:** `manifests/distributed/inference_policy.yaml`

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 50/60 (83.3%)
