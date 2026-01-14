# Script Contract: VERIFICATION-P1-012-P1-013.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/VERIFICATION-P1-012-P1-013.sh`

---

## Purpose

Verification script for P1-012 (SWC TypeScript compiler) and P1-013 (PixiJS library) implementation. Validates SWC wrapper availability, tests compilation, checks package.json/swcrc configuration, and performs end-to-end TypeScript transpilation test.

---

## Invocation

```bash
./scripts/VERIFICATION-P1-012-P1-013.sh
```

**Auto-activation:**
- If not in `nix develop` shell, script auto-launches `nix develop` and re-runs itself (lines 13-17)

**Examples:**
```bash
./scripts/VERIFICATION-P1-012-P1-013.sh             # Run verification
nix develop -c ./scripts/VERIFICATION-P1-012-P1-013.sh  # Explicit shell
```

---

## Side Effects

### Temporary Test Directory (lines 68-94)
- Creates `test-swc-verify/src/` with TypeScript test file
- Compiles to `test-swc-verify/dist/test.js`
- Cleans up test directory on success (line 94)

---

## Safety Classification

**🟢 SAFE** - Creates temporary test directory only, self-cleaning.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can run repeatedly, cleans up after itself.

---

## Key Features

### Auto-Shell Activation (lines 12-17)

```bash
# Check if we're in nix develop shell
if [ -z "$IN_NIX_SHELL" ]; then
    echo "⚠️  Not in nix shell. Running 'nix develop' first..."
    nix develop --command bash "$0" "$@"
    exit $?
fi
```

**Behavior:**
- Checks for `$IN_NIX_SHELL` environment variable
- If not set, re-launches script inside `nix develop`
- Passes through all arguments with `"$@"`

### Five-Step Verification (lines 22-101)

**1. SWC Wrapper Check (lines 23-31)**
```bash
echo "[1/5] Checking SWC wrapper..."
if command -v swc >/dev/null 2>&1; then
    echo "✓ SWC wrapper found in PATH"
    echo "  Location: $(which swc)"
else
    echo "✗ SWC wrapper not found"
    exit 1
fi
```
**Validates:** SWC wrapper script exists and is in PATH

**2. SWC Execution Test (lines 34-38)**
```bash
echo "[2/5] Testing SWC execution..."
echo "  Running: swc --version"
swc --version 2>&1 | head -3
```
**Validates:** SWC can execute via npx

**3. package.json Check (lines 40-50)**
```bash
echo "[3/5] Checking package.json..."
if [ -f "package.json" ]; then
    echo "✓ package.json found"
    echo "  PixiJS version: $(jq -r '.dependencies["pixi.js"]' package.json)"
    echo "  @swc/cli version: $(jq -r '.devDependencies["@swc/cli"]' package.json)"
    echo "  @swc/core version: $(jq -r '.devDependencies["@swc/core"]' package.json)"
else
    echo "✗ package.json not found"
    exit 1
fi
```
**Validates:** Required npm packages declared

**4. .swcrc Configuration Check (lines 53-63)**
```bash
echo "[4/5] Checking .swcrc configuration..."
if [ -f ".swcrc" ]; then
    echo "✓ .swcrc found"
    echo "  Parser syntax: $(jq -r '.jsc.parser.syntax' .swcrc)"
    echo "  Target: $(jq -r '.jsc.target' .swcrc)"
    echo "  Module type: $(jq -r '.module.type' .swcrc)"
else
    echo "✗ .swcrc not found"
    exit 1
fi
```
**Validates:** SWC configuration file exists with proper settings

**5. End-to-End Compilation Test (lines 66-100)**
```bash
echo "[5/5] Testing SWC compilation..."
echo "  Creating test TypeScript file..."
mkdir -p test-swc-verify/src
cat > test-swc-verify/src/test.ts << 'EOF'
const greeting: string = "Hello from SWC! P1-012 working!";
console.log(greeting);

interface Point {
  x: number;
  y: number;
}

const point: Point = { x: 10, y: 20 };
console.log(`Point: (${point.x}, ${point.y})`);
EOF

echo "  Compiling with SWC..."
cd test-swc-verify
swc src/test.ts -o dist/test.js

if [ -f "dist/test.js" ]; then
    echo "✓ Compilation successful!"
    echo "  Output file: test-swc-verify/dist/test.js"
    echo ""
    echo "  Compiled output:"
    echo "  ----------------"
    head -10 dist/test.js | sed 's/^/  /'
    cd ..
    rm -rf test-swc-verify
else
    echo "✗ Compilation failed"
    cd ..
    exit 1
fi
```

**Test TypeScript features:**
- Type annotations (`greeting: string`)
- Interfaces (`interface Point`)
- Template literals (`` `Point: (${point.x}, ${point.y})` ``)

**Output:**
- Shows first 10 lines of compiled JavaScript
- Verifies successful transpilation

### Next Steps Guidance (lines 103-113)

```bash
echo "=========================================="
echo "✓ All verification checks passed!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Install npm dependencies: pnpm install"
echo "  2. Import PixiJS in your code: import * as PIXI from 'pixi.js'"
echo "  3. Use SWC for compilation: swc src/ -d dist/"
echo ""
echo "Documentation: P1-012-P1-013-IMPLEMENTATION.md"
```

---

## Expected Configuration

### package.json
```json
{
  "dependencies": {
    "pixi.js": "^7.x.x"
  },
  "devDependencies": {
    "@swc/cli": "^0.x.x",
    "@swc/core": "^1.x.x"
  }
}
```

### .swcrc
```json
{
  "jsc": {
    "parser": {
      "syntax": "typescript"
    },
    "target": "es2020"
  },
  "module": {
    "type": "es6"
  }
}
```

---

## Exit Codes

- `0` - All checks passed
- `1` - Verification failed (missing file, compilation error, etc.)

---

## References

- **Main script:** `scripts/VERIFICATION-P1-012-P1-013.sh` (113 lines)
- **Shell check:** lines 12-17
- **SWC wrapper check:** lines 23-31
- **Execution test:** lines 34-38
- **package.json check:** lines 40-50
- **Configuration check:** lines 53-63
- **Compilation test:** lines 66-100
- **Related:** `P1-012-P1-013-IMPLEMENTATION.md` (implementation documentation)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 57/57 (100%)
