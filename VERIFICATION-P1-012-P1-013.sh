#!/usr/bin/env bash
# Verification script for P1-012 (SWC) and P1-013 (PixiJS) implementation
# Run this script to verify the build tools are properly configured

set -e

echo "=========================================="
echo "P1-012 & P1-013 Verification Script"
echo "=========================================="
echo ""

# Check if we're in nix develop shell
if [ -z "$IN_NIX_SHELL" ]; then
    echo "⚠️  Not in nix shell. Running 'nix develop' first..."
    nix develop --command bash "$0" "$@"
    exit $?
fi

echo "✓ Running in nix development environment"
echo ""

# 1. Verify SWC wrapper exists
echo "[1/5] Checking SWC wrapper..."
if command -v swc >/dev/null 2>&1; then
    echo "✓ SWC wrapper found in PATH"
    echo "  Location: $(which swc)"
else
    echo "✗ SWC wrapper not found"
    exit 1
fi
echo ""

# 2. Test SWC execution (will use npx)
echo "[2/5] Testing SWC execution..."
echo "  Running: swc --version"
swc --version 2>&1 | head -3
echo ""

# 3. Verify package.json exists
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
echo ""

# 4. Verify .swcrc configuration
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
echo ""

# 5. Test SWC compilation
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
echo ""

# Summary
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
