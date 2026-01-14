# Script Contract: build-frontend.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/build-frontend.sh`

---

## Purpose

Frontend asset builder using esbuild. Bundles TypeScript/JavaScript files with support for development/production modes, watch mode, minification, and source maps. Auto-creates example TypeScript entry point if source directory missing. Targets ES2020 browser platform.

---

## Invocation

```bash
./scripts/build-frontend.sh [OPTIONS]
```

**Options:**
- `--mode MODE` - Build mode: development or production (default: production)
- `--src DIR` - Source directory (default: src/)
- `--out DIR` - Output directory (default: dist/)
- `--watch` - Watch mode for development
- `--minify` - Minify output (default in production)
- `--sourcemap` - Generate source maps (default in production)
- `--help` - Show help

**Examples:**
```bash
./scripts/build-frontend.sh                           # Production build
./scripts/build-frontend.sh --mode development        # Development build
./scripts/build-frontend.sh --watch                   # Watch mode
./scripts/build-frontend.sh --mode production --minify --sourcemap
```

---

## Side Effects

### Output Directory (line 146)
- Creates `dist/` (or `$OUT_DIR`) with bundled JS files

### Example Source File (lines 112-123)
- Creates `src/index.ts` if source directory missing

---

## Safety Classification

**🟢 SAFE** - Creates output files only.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Overwrites output files.

---

## Key Features

### Production Mode Defaults (lines 92-95)
```bash
if [[ "$BUILD_MODE" == "production" ]]; then
    MINIFY=true
    SOURCEMAP=true
fi
```

### esbuild Command (lines 104-143)
```bash
esbuild $SRC_DIR/*.ts $SRC_DIR/*.tsx $SRC_DIR/*.js $SRC_DIR/*.jsx \
  --bundle \
  --outdir=$OUT_DIR \
  --minify \
  --sourcemap \
  --watch \
  --platform=browser \
  --target=es2020
```

### Auto-Generated Entry Point (lines 113-122)
```typescript
// Example TypeScript entry point
console.log("FlexStack Frontend Loaded");

export function init() {
    console.log("Initializing...");
}

init();
```

---

## References

- **Main script:** `scripts/build-frontend.sh` (162 lines)
- **Build command:** lines 104-155
- **Example source:** lines 112-123
- **External:** [esbuild documentation](https://esbuild.github.io/)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 47/60 (78.3%)
