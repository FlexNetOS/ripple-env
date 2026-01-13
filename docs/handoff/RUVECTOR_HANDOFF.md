# RuVector “Every Feature” + HTTP/gRPC Verification Handoff (Windows)

**Repo root:** `c:\Users\De-Flex.Net\source\repos\OKComputer_Re-Audit Ripple Environment\ripple-env`

This is a **comprehensive, self-contained checklist** to continue RuVector “every feature” + HTTP/gRPC verification work in this repo.

---

## 0) Current state snapshot (what’s true right now)

### Environment
- OS: Windows
- Node: `v20.18.1`
- npm: `10.8.2`
- `package.json` has `"engines": { "node": ">=22.0.0" }` → npm warns **EBADENGINE**, but installs and the RuVector CLI runs anyway.

### RuVector status (verified)

**CLI**
- `ruvector@0.1.96` works via `scripts/ruvector.ps1`.

**Feature modules**
- GNN Module: **Available** after pinning `@ruvector/gnn` to `0.1.19` on Windows.
- `graph --help` works (requires `@ruvector/graph-node`, installed).
- `attention info` shows **Available**.
- Router subcommand exists; may require `ruvector-router-core` (not necessarily installed).

**Server runtime is NOT available in this version/environment**
- `ruvector server --info` prints **“Status: Coming Soon”**.
- `ruvector server --http-only --port <port>` exits immediately with:
  - `Server package not yet available. Check issue #20 for roadmap.`
- `@ruvector/server@0.1.0` is installed but appears to be a stub (no actual runnable bin/payload present).

### Why GNN required a version pin
- `@ruvector/gnn@0.1.22` exists, but on Windows it references `@ruvector/gnn-win32-x64-msvc@0.1.22`, which is **not published**.
- Published native binding versions observed: `0.1.15`, `0.1.19`.
- Pinning `@ruvector/gnn` to `0.1.19` allows the Windows native binding to install, making GNN “Available”.

---

## 1) Files that were changed (important for the next agent)

### `package.json`
- Converted nonstandard `ruvectorDependencies` → standard `optionalDependencies` so npm installs RuVector modules into local `node_modules`.
- Pinned:
  - `@ruvector/gnn: 0.1.19` (Windows compatibility / published native binding)

### `scripts/cleanup-ruvector.ps1`
- Fix: PowerShell parsing bug using `${port}` (instead of `$port:`).
- Ensures it exits success (`exit 0`) even if best-effort cleanup hit non-terminating errors.

### `scripts/verify-ruvector.sh` and `scripts/verify-state-storage.sh`
- Updated messaging: server subcommand exists; runtime must be verified by starting/probing.
- Added optional strict checks in `verify-ruvector.sh`:
  - `RUVECTOR_REQUIRE_FULL=1` enforces GNN/graph availability.

### `scripts/ruvector.ps1` and `scripts/ruvector.sh`
- Prefer local `node_modules/.bin` binary first; fall back to `npx`.

---

## 2) “Do this first in a fresh chat” checklist (restore context fast)

### A) Sanity / cleanup (prevents terminal hang + port conflicts)
- Run: `powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\cleanup-ruvector.ps1`
- Confirm node/npm respond.

### B) Ensure repo-local npm cache/prefix is used
- Ensure `.npmrc` exists (it sets `cache=.npm-cache`).
- Wrappers set:
  - `NPM_CONFIG_CACHE=<repo>/.npm-cache`
  - `NPM_CONFIG_PREFIX=<repo>/.npm-prefix`

### C) Install deps into local `node_modules`
- Run (from repo root): `npm install --no-audit --no-fund`
- Expect EBADENGINE warnings until Node is upgraded to $\ge 22$ (optional decision).

### D) Verify modules are actually available
- `.\scripts\ruvector.ps1 --version`
- `.\scripts\ruvector.ps1 info`
- `.\scripts\ruvector.ps1 gnn info`
- `.\scripts\ruvector.ps1 graph --help`
- `.\scripts\ruvector.ps1 attention info`

Expected:
- `GNN Module: Available`

If GNN shows “Not installed”:
- Check `@ruvector/gnn` is still `0.1.19` in `package.json`.
- Check `node_modules/@ruvector/gnn-win32-x64-msvc/` exists.

---

## 3) HTTP/gRPC server runtime verification (what to do + expected outcome)

### Goal
Prove whether `ruvector server` can:
- bind an HTTP port, and
- respond to `/health` (or another health endpoint).

### Correct way to probe (non-blocking, no hangs)
- Start the server as a background process with stdout/stderr redirected to files.
- Probe endpoints with short timeouts.
- Kill server after probe.

### Expected outcome (current)
- Server exits quickly with: `Server package not yet available…`
- No `/health` endpoint responds.
- `server --info` says “Coming Soon”.

### Key note (important for future debugging)
Earlier hangs happened when trying to `ReadToEnd()` stdout of a long-running process.

Avoid that pattern; always redirect to files or tail logs.

---

## 4) Open decisions / next steps (choose one direction)

### Option 1 — “We need HTTP now”
Implement a small HTTP wrapper service inside this repo that exposes stable endpoints and calls RuVector locally.

Pick stack (choose the best quality and speed): Node (Express/Fastify/Hono) or Python (FastAPI) etc.

Minimal endpoints:
- `GET /health`
- `POST /vectors/insert`
- `POST /vectors/search`
- `GET /info` (returns `ruvector info` + module availability)

Under the hood:
- Use RuVector library APIs if available, else shell out to `ruvector` CLI.

Add:
- logging
- graceful shutdown
- config via `.env` (port, db path, etc.)
- tests: start server, hit `/health`, run a tiny insert+search flow

### Option 2 — “Server is real but not in this Windows npm channel”
Try to run RuVector server from WSL/Linux where packaging may differ.

- Check whether `ruvector server` behaves differently under WSL.
- If it binds ports and exposes health endpoints under WSL, document:
  - required versions
  - exact install steps
  - endpoints
  - how to run as a service

### Option 3 — “Track upstream / align versions”
If the project must stick to the RuVector “official server”:

- Track upstream Issue #20 and determine what “server” actually means today:
  - npm package?
  - separate Rust crate/binary (`ruvector-server`)?
- Add doc section:
  - “current reality vs target”
  - “how we’ll upgrade when server releases”

---

## 5) Tech-debt TODOs (recommended cleanup)

### Node engine mismatch
Decide:
- upgrade Node to $\ge 22$, OR
- relax `"engines.node"` in `package.json`.

Keeping it at $\ge 22$ is fine but noisy; Node 20 works with warnings currently.

### Dependency correctness / completeness
Confirm installed “elite” set in `package.json`:
- `ruvector`, `@ruvector/core`, `@ruvector/gnn` (pinned), `@ruvector/graph-node`, `@ruvector/attention`, `@ruvector/router`, `@ruvector/sona`, `@ruvector/tiny-dancer`, `ruvector-extensions`, `@ruvector/server` (currently stub)

Validate router’s required core package name (CLI help says `ruvector-router-core`).

### Lockfile policy
- A `package-lock.json` exists.
- `.gitignore` was previously updated to ignore `package-lock.json`.
- Verify whether it’s tracked and decide your policy.

### Verification scripts alignment
- Keep strict mode `RUVECTOR_REQUIRE_FULL=1` in `verify-ruvector.sh`.
- Optionally add a Windows-native equivalent verification script (PowerShell).

---

## 6) “Success criteria” for the project (clear acceptance targets)

### Feature completeness (local)
`ruvector info` shows:
- GNN: Available

Plus:
- `ruvector gnn info` shows “Available” feature list
- `ruvector graph --help` works
- `ruvector attention info` works

### Networked service (HTTP/gRPC)
Pick one:

A) Official RuVector server runs and responds to health checks, or

B) Repo-provided wrapper service provides HTTP endpoints and passes smoke tests

---

## 7) Handy commands

Cleanup:

Install:

Verify modules:
- `.\scripts\ruvector.ps1 info`
- `.\scripts\ruvector.ps1 gnn info`
- `.\scripts\ruvector.ps1 graph --help`
- `.\scripts\ruvector.ps1 attention info`

Server info:
- `.\scripts\ruvector.ps1 server --info`

Attempt server start (expected to fail in current channel):
- `.\scripts\ruvector.ps1 server --http-only --port 18080`

---

## Updated todo list status (for handoff)

- ✅ Unblock server runtime probe — completed (probe is clean; server runtime exits “not yet available”)
- ✅ Make modules resolvable locally — completed (GNN/graph/attention OK after Windows pin)
