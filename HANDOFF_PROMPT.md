# Handoff Prompt - Ripple-Env Continuation

> **Purpose**: This document provides context for continuing work in a new chat session.

---

## Project Context

**Repository**: `FlexNetOS/ripple-env`  
**Location**: `/home/ubuntu/ripple-env`  
**Technology Stack**: NixOS, Pixi, Rust, React Native (Expo), TypeScript

### Project Overview
Ripple-env is a development environment management system that includes:
- **NixOS configurations** for reproducible builds (stable/unstable)
- **Pixi package management** for Python/Node.js toolchains
- **Frontend applications**:
  - `vibe-kanban` - Rust+React kanban board for AI agent task management
  - `minimalist_sidebar_react` - Cross-platform (web/iOS/Android) Expo app
  - `agentic-flow` - Agent workflow visualization

---

## What Has Been Completed

### PR #51: Infrastructure Fixes (Ready for merge)
- ✅ Fixed Nix flake evaluation for `iso-ros2-stable`
- ✅ Removed invalid WSL overlay reference
- ✅ Deduplicated Docker configs via symlink
- ✅ Added h2 dependency to pixi.toml for HTTP/2 support
- ✅ Created documentation (FLAKE_FIXES_SUMMARY.md, NIX_FLAKE_TEST_REPORT.md)

### PR #52: Vibe-Kanban Integration Plan (Ready for review)
- ✅ Created comprehensive integration plan (VIBE_KANBAN_INTEGRATION_PLAN.md)
- ✅ Added kanban component scaffolding to minimalist_sidebar_react
- ✅ Added hooks for vibe-kanban API integration
- ✅ Added service layer with API client
- ✅ Updated sidebar navigation for kanban panel

---

## User's Original 6-Step Plan

### Completed Steps:
1. ✅ **Step 1**: Review existing changes (PR snapshot)
2. ✅ **Step 2**: Fix Nix flakes and document changes

### Remaining Steps (3-6):

#### Step 3: Vibe-Kanban Integration (IN PROGRESS)
**Goal**: Connect minimalist_sidebar_react to the EXISTING vibe-kanban app

**⚠️ CRITICAL**: Do NOT recreate vibe-kanban. Connect to the existing implementation:
- **Backend**: `/home/ubuntu/ripple-env/frontend/vibe-kanban/crates/server/`
- **API**: Axum-based REST API (see `src/routes/` for endpoints)
- **Database**: SQLite via SQLx

**Remaining Tasks**:
1. Start vibe-kanban backend server locally
2. Test API endpoints and document them
3. Wire up the scaffolded hooks to real API calls
4. Implement WebSocket connection for real-time updates
5. Test end-to-end task creation/updates

**API Base URL**: `http://localhost:3030` (default vibe-kanban port)

#### Step 4: Minimalist_Sidebar Sync
**Goal**: Bidirectional sync between minimalist_sidebar and vibe-kanban

**Requirements**:
- Sidebar panel state should reflect kanban task status
- Task updates from either UI should sync in real-time
- Maintain consistency across web/mobile platforms
- Handle offline scenarios gracefully

**Key Files to Modify**:
- `frontend/minimalist_sidebar_react/contexts/sidebar-context.tsx`
- `frontend/minimalist_sidebar_react/hooks/use-vibe-kanban.ts`
- `frontend/minimalist_sidebar_react/services/vibe-kanban.ts`

#### Step 5: Neovim + Lazyvim Integration
**Goal**: Enable task management from within Neovim

**Requirements**:
- Lua plugin for Lazyvim configuration
- Floating window for task list/kanban view
- Commands: `:KanbanOpen`, `:TaskCreate`, `:TaskUpdate`
- Integration with vibe-kanban API
- Optional: Telescope integration for task search

**Reference**: Look at vibe-kanban's CLI implementation for API patterns
- `frontend/vibe-kanban/npx-cli/` - NPX CLI for reference

#### Step 6: Testing & Deployment
**Goal**: Verify all integrations work correctly

**Tasks**:
- Unit tests for API client
- Integration tests for sync logic
- E2E tests for cross-platform functionality
- Performance testing for real-time updates
- Documentation updates

---

## Package Management Note

**Node modules should be installed via pixi package manager**, not npm/yarn/pnpm directly.

```bash
# Correct approach
pixi run -e llmops npm install <package>

# The llmops environment includes Node.js 22.x
pixi shell -e llmops
```

**Pixi configuration**: `pixi.toml` (root directory)

---

## Key File Locations

### Vibe-Kanban (Existing - Connect to this!)
```
frontend/vibe-kanban/
├── crates/server/src/
│   ├── routes/          # API endpoints
│   ├── handlers/        # Request handlers
│   └── main.rs          # Server entry
├── frontend/            # React frontend
├── shared/              # Shared TypeScript types
└── Cargo.toml           # Rust dependencies
```

### Minimalist Sidebar (Updated with scaffolding)
```
frontend/minimalist_sidebar_react/
├── app/(drawer)/
│   └── vibe-kanban.tsx         # Kanban route
├── components/kanban/          # Kanban components
├── hooks/
│   ├── use-vibe-kanban.ts     # Main hook
│   └── vibe-kanban/           # Specialized hooks
├── services/
│   └── vibe-kanban.ts         # API client
└── shared/vibe-kanban/
    └── types.ts               # TypeScript types
```

### Documentation
```
VIBE_KANBAN_INTEGRATION_PLAN.md  # Full integration blueprint
FLAKE_FIXES_SUMMARY.md           # Nix fix documentation
NIX_FLAKE_TEST_REPORT.md         # Test results
CONVERSATION_SUMMARY.md          # This session summary
```

---

## Immediate Next Actions

1. **Review and merge PRs** (if not already merged):
   - PR #51: https://github.com/FlexNetOS/ripple-env/pull/51
   - PR #52: https://github.com/FlexNetOS/ripple-env/pull/52

2. **Start vibe-kanban backend**:
   ```bash
   cd /home/ubuntu/ripple-env/frontend/vibe-kanban
   cargo run --bin server
   ```

3. **Test API connectivity**:
   ```bash
   curl http://localhost:3030/api/health
   curl http://localhost:3030/api/projects
   ```

4. **Wire up real API calls** in:
   - `services/vibe-kanban.ts` - Replace mock endpoints with real ones
   - `hooks/vibe-kanban/useTasks.ts` - Implement mutations

5. **Begin Neovim plugin development** (see Step 5 above)

---

## Git Branches

```bash
# Current state
main                              # Production
fix/nix-flake-docker-improvements # PR #51
feature/vibe-kanban-integration-plan # PR #52

# To continue work, create new branches from main after PRs merge:
git checkout main
git pull
git checkout -b feature/vibe-kanban-api-connection
```

---

## Environment Setup

```bash
# Enter development shell
cd /home/ubuntu/ripple-env
pixi shell -e llmops

# Or use Nix
nix develop
```

---

## Questions to Clarify with User

1. Should WebSocket be prioritized over polling for real-time sync?
2. Preference for Neovim plugin structure (single file vs. modular)?
3. Any specific mobile features to prioritize?
4. Preferred testing framework (Jest, Vitest, etc.)?

---

*Last Updated: January 15, 2026*
*Previous Session: Infrastructure fixes + Integration planning*
