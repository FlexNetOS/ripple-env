-- Configuration Database Schema
-- SQLite database for tracking flake inputs, workflows, and references

-- ============================================================================
-- FLAKE CONFIGURATION
-- ============================================================================

-- Flake inputs (from flake.nix inputs section)
CREATE TABLE IF NOT EXISTS flake_inputs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT UNIQUE NOT NULL,
    url TEXT NOT NULL,
    follows TEXT,              -- Which input it follows (e.g., "nixpkgs")
    locked_rev TEXT,           -- Commit hash from flake.lock
    locked_date TEXT,          -- Last modified date from flake.lock
    is_active INTEGER DEFAULT 1,
    created_at TEXT DEFAULT CURRENT_TIMESTAMP
);

-- DevShells defined in flake.nix
CREATE TABLE IF NOT EXISTS devshells (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT UNIQUE NOT NULL, -- default, full, cuda, identity
    description TEXT,
    platforms TEXT,            -- JSON array of supported platforms
    created_at TEXT DEFAULT CURRENT_TIMESTAMP
);

-- Packages included in each devshell
CREATE TABLE IF NOT EXISTS devshell_packages (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    devshell_id INTEGER REFERENCES devshells(id),
    package_name TEXT NOT NULL,
    package_source TEXT,       -- nixpkgs, overlay, pixi
    is_optional INTEGER DEFAULT 0,
    platform_condition TEXT,   -- linux, darwin, or null for all
    UNIQUE(devshell_id, package_name)
);

-- ============================================================================
-- GITHUB WORKFLOWS
-- ============================================================================

-- Workflow files
CREATE TABLE IF NOT EXISTS workflows (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    file_path TEXT UNIQUE NOT NULL,
    trigger_events TEXT,       -- JSON array: push, pull_request, etc.
    is_active INTEGER DEFAULT 1,
    created_at TEXT DEFAULT CURRENT_TIMESTAMP
);

-- Jobs within workflows
CREATE TABLE IF NOT EXISTS workflow_jobs (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    workflow_id INTEGER REFERENCES workflows(id),
    name TEXT NOT NULL,
    runs_on TEXT,              -- ubuntu-latest, macos-latest, etc.
    needs TEXT,                -- JSON array of job dependencies
    if_condition TEXT,
    UNIQUE(workflow_id, name)
);

-- Steps within jobs
CREATE TABLE IF NOT EXISTS workflow_steps (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    job_id INTEGER REFERENCES workflow_jobs(id),
    step_order INTEGER,
    name TEXT,
    uses_action TEXT,          -- e.g., actions/checkout@v4
    run_command TEXT,
    env_vars TEXT,             -- JSON object
    with_params TEXT           -- JSON object
);

-- Secrets referenced in workflows
CREATE TABLE IF NOT EXISTS workflow_secrets (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    workflow_id INTEGER REFERENCES workflows(id),
    secret_name TEXT NOT NULL,
    is_required INTEGER DEFAULT 1,
    used_in_job TEXT,
    UNIQUE(workflow_id, secret_name)
);

-- ============================================================================
-- REFERENCES AND CAPABILITIES
-- ============================================================================

-- Reference sources from README
CREATE TABLE IF NOT EXISTS reference_sources (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT UNIQUE NOT NULL,
    description TEXT,
    url TEXT NOT NULL,
    category TEXT,             -- Reference Sources, Agentic Infrastructure, etc.
    created_at TEXT DEFAULT CURRENT_TIMESTAMP
);

-- Capability Registry (API categories)
CREATE TABLE IF NOT EXISTS capability_categories (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    category_id TEXT UNIQUE NOT NULL,
    name TEXT NOT NULL,
    description TEXT,
    api_count INTEGER DEFAULT 0,
    path TEXT,
    use_cases TEXT             -- JSON array
);

-- ============================================================================
-- VALIDATION AND ISSUES
-- ============================================================================

-- Known issues and warnings
CREATE TABLE IF NOT EXISTS config_issues (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    source TEXT NOT NULL,      -- flake, workflow, reference
    severity TEXT NOT NULL,    -- error, warning, info
    issue_type TEXT,           -- unused_input, missing_secret, broken_link
    description TEXT NOT NULL,
    file_path TEXT,
    line_number INTEGER,
    is_resolved INTEGER DEFAULT 0,
    created_at TEXT DEFAULT CURRENT_TIMESTAMP,
    resolved_at TEXT
);

-- ============================================================================
-- VIEWS FOR COMMON QUERIES
-- ============================================================================

-- View: All flake inputs with their lock status
CREATE VIEW IF NOT EXISTS v_flake_inputs_status AS
SELECT
    name,
    url,
    follows,
    locked_rev,
    locked_date,
    CASE WHEN locked_rev IS NULL THEN 'UNLOCKED' ELSE 'LOCKED' END as lock_status,
    is_active
FROM flake_inputs;

-- View: Workflow summary
CREATE VIEW IF NOT EXISTS v_workflow_summary AS
SELECT
    w.name as workflow_name,
    w.file_path,
    COUNT(DISTINCT j.id) as job_count,
    COUNT(DISTINCT s.id) as secret_count,
    w.trigger_events
FROM workflows w
LEFT JOIN workflow_jobs j ON w.id = j.workflow_id
LEFT JOIN workflow_secrets s ON w.id = s.workflow_id
GROUP BY w.id;

-- View: Unresolved issues by severity
CREATE VIEW IF NOT EXISTS v_unresolved_issues AS
SELECT
    severity,
    source,
    issue_type,
    description,
    file_path
FROM config_issues
WHERE is_resolved = 0
ORDER BY
    CASE severity WHEN 'error' THEN 1 WHEN 'warning' THEN 2 ELSE 3 END,
    created_at DESC;

-- ============================================================================
-- INDEXES FOR PERFORMANCE
-- ============================================================================

CREATE INDEX IF NOT EXISTS idx_flake_inputs_name ON flake_inputs(name);
CREATE INDEX IF NOT EXISTS idx_workflows_file ON workflows(file_path);
CREATE INDEX IF NOT EXISTS idx_issues_resolved ON config_issues(is_resolved);
CREATE INDEX IF NOT EXISTS idx_issues_severity ON config_issues(severity);
