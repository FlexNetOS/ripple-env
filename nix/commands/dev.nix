# Development workflow command wrappers
# Includes: fmt-nix, dev-check, pre-commit, swc
# Extracted from flake.nix for modular organization
{ pkgs, ... }:

[
            # Format all Nix files
            (pkgs.writeShellScriptBin "fmt-nix" ''
              # Format all Nix files in the repository
              echo "Formatting Nix files..."
              find . -name "*.nix" -type f ! -path "./.git/*" -exec nixfmt {} +
              echo "Done."
            '')
            # Run all checks
            (pkgs.writeShellScriptBin "dev-check" ''
              # Run all development checks
              # Usage: dev-check [--fix]
              set -e
              FIX=""
              [ "$1" = "--fix" ] && FIX="1"

              echo "Running development checks..."
              echo "=============================="
              FAILED=0

              # Nix formatting
              echo ""
              echo "[1/4] Checking Nix formatting..."
              if [ -n "$FIX" ]; then
                find . -name "*.nix" -type f ! -path "./.git/*" -exec nixfmt {} + && echo "  Formatted."
              else
                if find . -name "*.nix" -type f ! -path "./.git/*" -exec nixfmt --check {} + 2>/dev/null; then
                  echo "  OK"
                else
                  echo "  FAIL - Run with --fix to auto-format"
                  FAILED=1
                fi
              fi

              # Nix flake check
              echo ""
              echo "[2/4] Checking Nix flake..."
              if command -v nix >/dev/null 2>&1; then
                if nix flake check --no-build 2>&1 | head -20; then
                  echo "  OK"
                else
                  echo "  FAIL"
                  FAILED=1
                fi
              else
                echo "  SKIP (nix not available)"
              fi

              # Pixi lock check
              echo ""
              echo "[3/4] Checking pixi.lock..."
              if [ -f pixi.toml ]; then
                if [ -f pixi.lock ]; then
                  echo "  OK (lock file exists)"
                else
                  echo "  FAIL - Run 'pixi install' to create lock file"
                  FAILED=1
                fi
              else
                echo "  SKIP (no pixi.toml)"
              fi

              # ROS2 build check
              echo ""
              echo "[4/4] Checking ROS2 packages..."
              if [ -d "src" ] && command -v colcon >/dev/null 2>&1; then
                if colcon build --packages-select-build-failed 2>&1 | grep -q "No packages"; then
                  echo "  OK (all packages build)"
                else
                  echo "  WARN - Some packages may have build issues"
                fi
              else
                echo "  SKIP (no src/ or colcon unavailable)"
              fi

              echo ""
              echo "=============================="
              if [ "$FAILED" -eq 0 ]; then
                echo "All checks passed!"
              else
                echo "Some checks failed. Run with --fix to auto-fix where possible."
                exit 1
              fi
            '')
            # Git pre-commit helper
            (pkgs.writeShellScriptBin "pre-commit" ''
              # Run pre-commit checks
              # Install as git hook: ln -sf $(which pre-commit) .git/hooks/pre-commit
              echo "Running pre-commit checks..."

              # Check for large files
              LARGE_FILES=$(git diff --cached --name-only | xargs -I{} sh -c 'test -f "{}" && du -k "{}" | awk "\$1 > 1024 {print \$2}"' 2>/dev/null)
              if [ -n "$LARGE_FILES" ]; then
                echo "Warning: Large files (>1MB) staged for commit:" >&2
                echo "$LARGE_FILES" | sed 's/^/  /' >&2
                echo "Consider using Git LFS for large files." >&2
              fi

              # Check for secrets patterns
              if git diff --cached | grep -E "(password|secret|api_key|private_key)\s*[:=]" >/dev/null 2>&1; then
                echo "Warning: Possible secrets detected in staged changes" >&2
                echo "Please review before committing." >&2
              fi

              # Format staged Nix files
              STAGED_NIX=$(git diff --cached --name-only --diff-filter=ACM | grep '\.nix$' || true)
              if [ -n "$STAGED_NIX" ]; then
                echo "Formatting staged Nix files..."
                echo "$STAGED_NIX" | xargs nixfmt
                echo "$STAGED_NIX" | xargs git add
              fi

              echo "Pre-commit checks complete."
            '')
            # P1-012: SWC compiler wrapper
            # Fast TypeScript/JavaScript compiler written in Rust
            # 10x faster than tsc for transpilation and bundling
            (pkgs.writeShellScriptBin "swc" ''
              # SWC (Speedy Web Compiler) - Fast TypeScript/JavaScript compiler
              # Usage: swc <input> [options]
              # Examples:
              #   swc src/index.ts -o dist/index.js
              #   swc src/ -d dist/
              #
              # Uses npx to avoid 150MB node_modules bloat
              # See: https://github.com/NixOS/nixpkgs/issues/195677
              exec npx -y @swc/cli@latest "$@"
            '')
          ];
