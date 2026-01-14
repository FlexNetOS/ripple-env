# Script Contract: check-python-deps.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/check-python-deps.sh`

---

## Purpose

Python dependency conflict detector for Pixi environments. Validates Python version constraints (AIOS requires 3.10-3.11), PyTorch ecosystem version coupling (torchvision/torchaudio), NumPy compatibility (warns on 2.x), pydantic version requirements (AIOS needs 2.7.0), and tests critical package imports. Provides resolution steps for detected conflicts.

---

## Invocation

```bash
./scripts/check-python-deps.sh [environment|--all]
```

**Arguments:**
- (no argument) - Check default environment
- `environment` - Check specific environment (cuda, aios, llmops, etc.)
- `--all` - Check all environments

**Examples:**
```bash
./scripts/check-python-deps.sh          # Check default
./scripts/check-python-deps.sh cuda     # Check CUDA environment
./scripts/check-python-deps.sh aios     # Check AIOS environment
./scripts/check-python-deps.sh --all    # Check all environments
```

**Exit Codes:**
- `0` - No conflicts
- `1` - Conflicts detected
- `2` - Script error

---

## Side Effects

**None** - Read-only validation.

---

## Safety Classification

**🟢 SAFE** - Read-only checks.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can run repeatedly.

---

## Validation Checks

### 1. Python Version (lines 48-76)
- **AIOS:** Requires Python 3.10-3.11 (AGiXT constraint)
- **Default:** Recommends Python 3.11-3.12 (RoboStack compatibility)

### 2. PyTorch Coupling (lines 78-141)
```python
COUPLING = {
    '2.5': {'torchvision': '0.20', 'torchaudio': '2.5'},
    '2.4': {'torchvision': '0.19', 'torchaudio': '2.4'},
    '2.3': {'torchvision': '0.18', 'torchaudio': '2.3'},
}
```
**Validates:** torchvision and torchaudio versions match PyTorch major.minor.

### 3. NumPy Compatibility (lines 143-173)
**Warns:** NumPy 2.x may break ROS2 and ML packages.

**Recommendation:** Pin `numpy = "<2"` in pixi.toml.

### 4. pydantic Version (lines 175-206)
**AIOS requirement:** Exactly pydantic==2.7.0 (AGiXT agent-core constraint).

### 5. Import Tests (lines 208-258)
**Critical packages:**
- numpy, pandas, torch, transformers, rclpy

**Optional packages:**
- accelerate, sentence_transformers, mlflow, chromadb

---

## Example Output

```
==========================================
Python Dependency Conflict Checker
==========================================

========================================
Environment: aios
========================================

[INFO] Checking Python version for environment: aios
[OK] Python 3.11 is compatible with AIOS

[INFO] Checking PyTorch version coupling for environment: aios
PyTorch: 2.5.0 (major.minor: 2.5)
TorchVision: 0.20.1 (major.minor: 0.20)
TorchAudio: 2.5.1 (major.minor: 2.5)
OK: PyTorch ecosystem versions are correctly coupled

[INFO] Checking NumPy compatibility for environment: aios
NumPy version: 1.26.4
OK: NumPy 1.x is widely compatible

[INFO] Checking pydantic version for environment: aios
pydantic version: 2.7.0
OK: pydantic v2 in use

[INFO] Testing critical package imports for environment: aios
Critical packages:
  numpy: 1.26.4
  pandas: 2.2.3
  torch: 2.5.0
  transformers: 4.46.3
  rclpy: 7.4.1

Optional packages:
  accelerate: 1.2.1
  sentence_transformers: 3.3.1
  mlflow: 2.18.0
  chromadb: not installed (optional)

==========================================
[OK] No conflicts detected
```

---

## References

- **Main script:** `scripts/check-python-deps.sh` (335 lines)
- **Python version check:** lines 48-76
- **PyTorch coupling:** lines 78-141
- **NumPy compat:** lines 143-173
- **pydantic check:** lines 175-206
- **Import tests:** lines 208-258
- **Related:** `pixi.toml` (dependency specifications)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 48/60 (80%)
