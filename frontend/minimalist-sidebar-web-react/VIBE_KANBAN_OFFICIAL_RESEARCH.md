# Vibe-Kanban Official Research - Phase 1

## Source: https://www.vibekanban.com/

### Core Value Proposition
**"Orchestrate AI Coding Agents"** - Vibe Kanban is fundamentally an **AI coding agent orchestration platform**, NOT just a task management tool.

### Key Differentiators Found

#### 1. **AI Coding Agent Integration** (PRIMARY FEATURE)
- Works with multiple AI coding agents:
  - Claude Code
  - OpenAI Codex
  - GitHub Copilot
  - Amp
  - Gemini CLI
  - Cursor CLI
  - Opencode
  - Qwen Code
  - Factory Droid

#### 2. **Parallel Agent Execution**
- Run multiple coding agents simultaneously
- Each agent works in separate git worktrees
- No conflicts between agents
- Focus on planning while agents work in background

#### 3. **Git Worktree Integration**
- Agents work in separate branches
- Powered by git worktrees
- Safe merging when tasks complete
- Prevents agents from stepping on each other's toes

#### 4. **Built-in Code Review / Diff Tool**
- Inspect agent changes like pull requests
- Edit and approve agent work
- Built-in diff visualization

#### 5. **Real-World Usage Examples**
From the homepage, actual tasks being completed:
- "add orgType field to the project" (zilliztech/terraform-provider-zillizcloud)
- "html skill" (ChatTreeNet/PhoenixClaw) - converting openclaw info to images
- "Fix breaking bugs" (AndreJorgeLopes/casemove-unlocked) - fixing TypeScript errors

### Installation
```bash
npx vibe-kanban
```
Requires: Node.js 18+

### Version
- Latest: v0.1.11 (released 4 hours ago from research time)
- Active development with frequent releases

### Community
- 21k GitHub stars
- Used by "thousands of top developers"
- Hiring actively

### User Testimonials
- "Biggest increase in productivity since Cursor" - Luke Harries (Growth Lead at Eleven Labs)
- "This project is underrated. I've found it to be useful and fun" - Hamel Husain

---

## Critical Insight: FUNDAMENTAL MISALIGNMENT

**Our implementation is a TASK MANAGEMENT system.**
**Vibe-Kanban is an AI CODING AGENT ORCHESTRATION platform.**

These are fundamentally different products:
- **Our system**: Manage human tasks, subtasks, time tracking, comments
- **Vibe-Kanban**: Orchestrate AI agents to write code in parallel

### What This Means
We need to research the actual KANBAN BOARD UI/UX features, not the AI orchestration features.

---

## Next Steps
1. Navigate to documentation to understand the kanban board UI
2. Look at interface guide
3. Check GitHub repo for UI components
4. Identify any kanban-specific features we're missing

*Research continues...*


---

## Kanban Board Features (Cloud Beta)

### Source: https://vibekanban.com/docs/cloud/kanban-board

### Board Layout Components
1. **Filter Bar** (top) - Search, filter, and sort issues
2. **Status Columns** - Vertical columns for workflow stages
3. **Issue Cards** - Individual work items
4. **Column Headers** - Status name with color indicator

### Default Status Columns (6 total)
1. **To do** - Ready to start
2. **In progress** - Currently being worked on  
3. **In review** - Waiting for code review
4. **Done** - Completed
5. **Backlog** (hidden by default) - Ideas and future work
6. **Cancelled** (hidden by default) - Issues that won't be done

**Note:** Only 4 columns visible by default. Backlog and Cancelled are hidden.

### Issue Card Components
- **Simple ID** (e.g., ISS-1)
- **Title** - What needs to be done
- **Priority** - Color-coded indicator
- **Assignees** - Avatar(s) of assigned team members
- **Tags** - Colored labels

### Drag and Drop Features
1. **Move Between Columns** - Change issue status
2. **Reorder Within Column** - Only in "Manual" sort mode
3. **Visual Feedback**:
   - Cursor changes
   - Drop target highlights
   - Card shadow

### Status Tabs (Above Board)
- **Active Kanban** - Kanban columns for visible statuses
- **All** - List view of all issues including hidden statuses
- **Backlog, Cancelled, etc.** - List view of specific hidden status

### Real-Time Collaboration
- Issue moved - Cards animate to new position
- Issue created - New cards appear automatically
- Issue updated - Changes appear instantly
- Issue deleted - Cards disappear
- **No refresh needed** - Everything syncs automatically

### Keyboard Shortcuts
- `Escape` - Close issue panel
- `Cmd/Ctrl + K` - Open command bar

### Best Practices (from docs)
1. **Limit work in progress** - Don't have too many in "In Progress"
2. **Review regularly** - Use in daily standups
3. **Keep columns meaningful** - Each column = real workflow stage
4. **Use filters for focus** - Filter by assigned, priority, tags

---

## Interface Features (Workspaces UI)

### Source: https://www.vibekanban.com/docs/workspaces/interface

### Four-Panel Layout
1. **Workspace Sidebar** (left edge) - List of all workspaces with status indicators
2. **Conversation Panel** (left main) - Chat with coding agents
3. **Context Panel** (right main) - Changes, logs, or preview (toggleable)
4. **Details Sidebar** (right edge) - Git status, terminal, notes

### Workspace Status Indicators
- **Running** - Agent actively processing
- **Idle** - Waiting for input
- **Needs Attention** - Pending approval (raised hand icon)
- **Pinned** - Pinned to top
- **Dev Server** - Blue indicator when running
- **PR Status** - Badge showing linked pull request

### Workspace Actions
- **Search** - Filter by name or branch
- **Pin** - Keep at top
- **Archive** - Move completed out of main list
- **Layout toggle** - Flat list vs accordion (grouped by status)

### Chat Shortcuts
- `Cmd/Ctrl + Enter` - Send message
- `Shift + Cmd/Ctrl + Enter` - Alternative send mode
- `Cmd/Ctrl + B` - Bold text
- `Cmd/Ctrl + I` - Italic text
- `Cmd/Ctrl + U` - Underline text

### Context Panel Views
1. **Changes View** - File tree, diff viewer, inline comments
2. **Logs View** - Process execution logs with search
3. **Preview View** - Built-in browser for testing

### Details Sidebar Sections
1. **Git Section** - Current repo, branch, target branch, commits ahead/behind
2. **Terminal Section** - Full terminal emulation (xterm.js)
3. **Notes Section** - Per-workspace notes with auto-save

### Context Bar (Floating Toolbar)
- Open in IDE
- Copy Path
- Toggle Dev Server
- Toggle Preview
- Toggle Changes
- **Draggable** - Position anywhere

---

## Key Findings: What We're Missing

### Missing from Our Implementation:

#### 1. **Backlog Status** ❌
- Vibe-kanban has 6 statuses (we have 5)
- Backlog is hidden by default but accessible
- We have: Todo, In Progress, Review, Done, Cancelled
- **Missing: Backlog**

#### 2. **Status Tabs Above Board** ❌
- Active Kanban view
- All (list view)
- Individual hidden status tabs
- We don't have tab navigation

#### 3. **Manual vs Auto Sort Mode** ❌
- Vibe-kanban has "Manual" sort mode for drag reordering
- Auto sort modes: Priority, Created, etc.
- We need to disable drag when in auto-sort mode

#### 4. **Real-Time Collaboration** ❌
- Live updates when team members make changes
- Animated card movements
- No refresh needed
- We don't have WebSocket/real-time sync

#### 5. **Issue ID System** ❌
- Simple IDs like "ISS-1", "ISS-2"
- We use database auto-increment IDs
- Need human-readable task IDs

#### 6. **Empty Column States** ❌
- Show empty state message
- + button to create issue in that column
- We have + button but no empty state design

#### 7. **Filter Bar** ⚠️ (Partially implemented)
- We have search and filters
- Missing: Sort dropdown in filter bar
- Missing: Visual filter chips

#### 8. **Column Customization** ❌
- Add new columns
- Rename columns
- Change colors
- Hide/show columns
- We have fixed columns

#### 9. **WIP Limits** ❌
- Work-in-progress limits per column
- Visual indicators when limit exceeded

#### 10. **Issue Panel** ❌
- Opens on right side (not modal)
- Stays open while browsing board
- We use modal dialogs

---

## Additional Features from Workspaces UI

### Not Applicable to Task Management:
- AI coding agent integration
- Git worktree management
- Code diff viewer
- Terminal integration
- Dev server preview
- PR tracking

### Potentially Applicable:
- **Four-panel layout** - Could adapt for task management
- **Status indicators** - Running, Idle, Needs Attention
- **Pinning** - Pin important tasks
- **Notes section** - Per-task notes
- **Real-time collaboration** - Team updates

---

## Next Steps

1. Navigate to GitHub repo to see actual UI components
2. Look for screenshots/videos of the kanban board
3. Check for any additional features in the codebase
4. Identify priority features to implement

*Research continues...*


---

## GitHub Repository Analysis

### Source: https://github.com/BloopAI/vibe-kanban

### Repository Structure
- **Frontend**: TypeScript (52.4%) + React + Vite
- **Backend**: Rust (45.3%) + Axum + SQLite
- **Styling**: TailwindCSS + CSS (0.9%)
- **21k stars**, 2k forks, 277 open issues, 103 PRs

### Key Commits & Features
1. **Remote kanban (#2416)** - Kanban layout mode with toggle
2. **feat: share tasks (#1210)** - Task sharing functionality
3. **Workspaces FE (#1733)** - Workspaces UI implementation
4. **Louis/default cloud kanban (#2674)** - Cloud kanban features
5. **Fix onboarding & migration flow UX issues (#2698)** - Recent UX improvements

### Notable Features from Commits
- **Slash commands (#2193)** - Command interface
- **feat: display context usage in UI for Codex and Claude Code (#1775)** - AI agent context tracking
- **feat(vscode): VS Code extension UI improvements (#2400)** - IDE integration

---

## FINAL RESEARCH SUMMARY

### Critical Understanding

**Vibe-Kanban is NOT a traditional task management tool.** It's an **AI coding agent orchestration platform** that happens to use a kanban board interface to visualize and manage AI agent tasks.

### Core Differences

| Aspect | Traditional Task Manager | Vibe-Kanban |
|--------|-------------------------|-------------|
| **Primary User** | Humans | AI Coding Agents |
| **Task Type** | Business/project tasks | Code changes/features |
| **Execution** | Manual human work | Automated AI agent work |
| **Status** | Human progress | Agent execution state |
| **Review** | Team review | Code diff review |
| **Workspace** | Project boards | Git worktrees |

### What We Should Implement (Kanban UI Features Only)

#### Priority 1: Core Kanban Features ✅ (Already have most)
1. ✅ Drag and drop between columns
2. ✅ Task cards with metadata
3. ✅ Multiple status columns
4. ✅ Search and filters
5. ✅ Task detail view

#### Priority 2: Missing Kanban Features (Should Add)
1. ❌ **Backlog status** - 6th column (hidden by default)
2. ❌ **Status tabs** - Active Kanban, All, Backlog, Cancelled tabs
3. ❌ **Manual vs Auto sort** - Disable drag in auto-sort mode
4. ❌ **Human-readable IDs** - ISS-1, ISS-2 format
5. ❌ **Empty column states** - Better UX for empty columns
6. ❌ **Column customization** - Add/rename/hide columns
7. ❌ **Issue panel (side)** - Right-side panel instead of modal
8. ❌ **Real-time collaboration** - WebSocket live updates
9. ❌ **WIP limits** - Per-column work-in-progress limits
10. ❌ **Filter chips** - Visual active filter indicators

#### Priority 3: Not Applicable (AI Agent Specific)
- Git worktree integration
- AI coding agent execution
- Code diff viewer
- Terminal integration
- Dev server preview
- MCP configuration

---

## IMPLEMENTATION PLAN

### Phase 1: Add Missing Core Features
1. Add Backlog status (6th column, hidden by default)
2. Implement status tabs (Active Kanban, All, Backlog, Cancelled)
3. Add manual vs auto-sort mode toggle
4. Implement human-readable task IDs (TASK-1, TASK-2)
5. Improve empty column states

### Phase 2: Enhanced UX
1. Convert task detail modal to side panel
2. Add column customization (add/rename/hide/reorder)
3. Implement WIP limits with visual indicators
4. Add filter chips showing active filters
5. Improve drag feedback and animations

### Phase 3: Collaboration
1. Implement WebSocket for real-time updates
2. Show active users viewing board
3. Live cursor positions
4. Conflict resolution for simultaneous edits

### Phase 4: Advanced Features
1. Keyboard navigation (arrow keys, shortcuts)
2. Bulk operations (multi-select, bulk edit)
3. Board templates
4. Custom fields
5. Advanced analytics

---

## END-TO-END TESTING PLAN

### Test Categories

#### 1. **Backend API Testing**
- ✅ Verify all 33 endpoints are accessible
- ✅ Test CRUD operations for tasks
- ✅ Test comments, activities, time entries
- ✅ Test AI features endpoints
- ✅ Test recurring tasks endpoints
- ✅ Verify database schema integrity

#### 2. **Frontend Integration Testing**
- ✅ Verify API calls are properly connected
- ✅ Test loading states
- ✅ Test error handling
- ✅ Test optimistic updates
- ✅ Verify data persistence

#### 3. **UI/UX Testing**
- ✅ Test drag and drop functionality
- ✅ Test all modals and dialogs
- ✅ Test filters and sorting
- ✅ Test search functionality
- ✅ Test responsive design
- ✅ Test keyboard shortcuts

#### 4. **Feature Completeness Testing**
- ✅ Verify all vibe-kanban-inspired features work
- ✅ Test subtasks functionality
- ✅ Test recurring tasks
- ✅ Test AI suggestions
- ✅ Test time tracking
- ✅ Test comments and activity feed

#### 5. **Performance Testing**
- Test with 100+ tasks
- Test with multiple users
- Test real-time updates
- Test database query performance

---

## NEXT ACTIONS

1. ✅ Complete research (DONE)
2. ⏳ Implement Priority 1 missing features
3. ⏳ Run comprehensive end-to-end testing
4. ⏳ Fix all identified issues
5. ⏳ Document final delivery

*Research complete. Moving to implementation phase.*
