# ARIA MCP Tool Schemas - Index

**Version**: 1.0.0
**Last Updated**: 2026-01-09
**Schema Specification**: JSON Schema Draft 2020-12

---

## Schema Catalog

### 1. Repository Analysis Tool
**File**: `repository-analysis.schema.json`
**Schema ID**: `https://aria.tools/mcp/schemas/repository-analysis`
**Version**: 1.0.0

**Purpose**: Analyzes code repositories for ARIA integration

**Input Parameters**:
- `repository_url` (required): GitHub repository URL
- `analysis_depth`: shallow | standard | deep
- `focus_areas`: Array of analysis focuses
- `target_domain`: One of 20 ARIA domains

**Output**:
- Repository metadata
- Dependencies (system, runtime, development)
- Installation method recommendations
- Configuration requirements
- Integration points
- Conflict detection
- Verification commands

**Use Cases**:
- Domain repository discovery
- Installation planning
- Dependency mapping
- Integration feasibility assessment

---

### 2. Domain Orchestration Tool
**File**: `domain-orchestration.schema.json`
**Schema ID**: `https://aria.tools/mcp/schemas/domain-orchestration`
**Version**: 1.0.0

**Purpose**: Orchestrates parallel analysis across ARIA's 20 architectural domains

**Input Parameters**:
- `domains`: Array of domains to analyze (or "all")
- `analysis_mode`: discovery | installation | verification | comprehensive
- `parallel_limit`: Max parallel domain analyses
- `output_format`: json | markdown | yaml

**Output**:
- Execution summary
- Per-domain results with status
- Cross-domain analysis (shared deps, integration points)
- Feature flags needed
- Aggregated tasks by priority (P0-P3)
- Recommendations

**Use Cases**:
- Multi-domain comprehensive analysis
- Cross-domain dependency identification
- Conflict matrix generation
- Task backlog creation

---

### 3. Configuration Manager Tool
**File**: `configuration-manager.schema.json`
**Schema ID**: `https://aria.tools/mcp/schemas/configuration-manager`
**Version**: 1.0.0

**Purpose**: Manages configuration files across ARIA's multi-platform stack

**Input Parameters**:
- `action`: validate | detect_drift | propose_update | apply_update | rollback
- `config_files`: Array of config file paths and types
- `validation_rules`: Syntax, schema, dependencies, security
- `update_spec`: Component, version, dependencies, configuration

**Output**:
- Per-file validation results
- Drift detection details
- Proposed/applied changes with diffs
- Security issues and remediation
- Backup paths
- Summary statistics
- Recommendations

**Supported Formats**:
- flake.nix (Nix)
- pixi.toml (Pixi)
- docker-compose (Docker)
- Cargo.toml (Rust)
- package.json (Node)
- YAML, JSON

**Use Cases**:
- Configuration validation
- Drift detection and remediation
- Safe configuration updates
- Rollback capabilities

---

### 4. Workflow Verifier Tool
**File**: `workflow-verifier.schema.json`
**Schema ID**: `https://aria.tools/mcp/schemas/workflow-verifier`
**Version**: 1.0.0

**Purpose**: Verifies CI/CD workflows and integration flows

**Input Parameters**:
- `verification_type`: workflow_syntax | pipeline_execution | smoke_tests | integration_tests | e2e_flow | comprehensive
- `workflow_paths`: Paths to workflow files
- `target_environment`: local | ci | staging | production
- `test_scope`: Domains, components, include_optional
- `verification_options`: Timeout, parallel, fail_fast, report generation

**Output**:
- Verification summary with statistics
- Workflow results (syntax, job execution)
- Smoke test results
- Integration test results
- E2E flow analysis with bottlenecks
- Recommendations
- Report path

**Use Cases**:
- CI/CD workflow validation
- Smoke testing after deployments
- Integration flow verification
- Performance bottleneck detection

---

### 5. Feature Flag Manager Tool
**File**: `feature-flag-manager.schema.json`
**Schema ID**: `https://aria.tools/mcp/schemas/feature-flag-manager`
**Version**: 1.0.0

**Purpose**: Manages A/B feature flags to resolve component conflicts

**Input Parameters**:
- `action`: create | list | update | delete | switch | validate
- `flag_definition`: Flag name, conflict area, options, default, config locations
- `switch_spec`: Flag name, target option, apply changes, backup

**Output**:
- Action result status
- Flag listings with current state
- Switch results with affected files
- Verification (config valid, deps resolved, health checks)
- Rollback information
- Conflict matrix
- Recommendations

**ARIA Philosophy**: Never omit features—use A/B flags for conflict resolution

**Use Cases**:
- Component conflict resolution
- A/B configuration switching
- Feature experimentation
- Safe rollback mechanisms

---

### 6. Dependency Resolver Tool
**File**: `dependency-resolver.schema.json`
**Schema ID**: `https://aria.tools/mcp/schemas/dependency-resolver`
**Version**: 1.0.0

**Purpose**: Resolves dependencies across ARIA's multi-platform stack

**Input Parameters**:
- `action`: analyze | detect_conflicts | resolve | generate_install_order | validate
- `scope`: Domains, components, include_transitive
- `platforms`: Array of platforms (or "all")
- `resolution_strategy`: conservative | balanced | aggressive | latest
- `constraints`: Min/max/pinned/excluded versions

**Output**:
- Dependency graph (nodes, edges, statistics)
- Conflicts with resolution options
- Circular dependencies
- Missing dependencies
- Resolved dependencies by platform
- Installation order with commands
- Validation results (including security)
- Recommendations

**Platforms**:
- Nix, Pixi, Docker, Cargo, NPM, Pip, System

**Use Cases**:
- Dependency graph analysis
- Version conflict resolution
- Installation order planning
- Security vulnerability detection

---

### 7. Agent Capability Mapper Tool
**File**: `agent-capability-mapper.schema.json`
**Schema ID**: `https://aria.tools/mcp/schemas/agent-capability-mapper`
**Version**: 1.0.0

**Purpose**: Maps agent capabilities to available tools and APIs

**Input Parameters**:
- `action`: discover | map | search | recommend | validate
- `agent_profile`: Agent ID, type, domain, required/optional capabilities, constraints
- `task_requirements`: Task type, capabilities needed, formats, performance requirements
- `search_query`: Categories, keywords, use cases, filters

**Output**:
- Capability map (capabilities → tools with confidence scores)
- Discovered tools with integration methods
- Search results with relevance scoring
- Recommendations with pros/cons
- Validation results with gap analysis
- Registry statistics

**Integration**: Uses 10,498-API capability registry from Apify

**Use Cases**:
- Dynamic tool discovery
- Agent capability mapping
- Tool recommendation
- Gap analysis

---

## Schema Validation

### Validation Command

```bash
# Using jsonschema CLI (Python)
pip install jsonschema-cli

# Validate a schema
jsonschema -i repository-analysis.schema.json

# Validate all schemas
for schema in *.schema.json; do
    echo "Validating $schema..."
    jsonschema -i "$schema" || echo "FAILED: $schema"
done
```

### Validation Checklist

All schemas must:
- [ ] Follow JSON Schema Draft 2020-12 specification
- [ ] Include `$schema`, `$id`, `title`, `description`, `version`
- [ ] Define `name` (tool identifier)
- [ ] Define `inputSchema` with required fields
- [ ] Define `outputSchema` with required fields
- [ ] Include comprehensive examples
- [ ] Document all enum values
- [ ] Specify formats for strings (uri, date-time, etc.)
- [ ] Include descriptions for all fields
- [ ] Follow ARIA naming conventions

---

## Usage Patterns

### Pattern 1: Sequential Tool Chain

```python
# Example: Analyze repository → Resolve dependencies → Generate install order

# Step 1: Analyze repository
repo_result = repository_analysis_tool.execute({
    "repository_url": "https://github.com/agixt/agixt",
    "analysis_depth": "standard",
    "target_domain": "Agent Runtime"
})

# Step 2: Resolve dependencies
dep_result = dependency_resolver_tool.execute({
    "action": "resolve",
    "scope": {
        "components": [repo_result["repository"]["name"]],
        "include_transitive": True
    },
    "platforms": ["docker", "pip"],
    "resolution_strategy": "balanced"
})

# Step 3: Generate installation order
install_order = dependency_resolver_tool.execute({
    "action": "generate_install_order",
    "resolved_dependencies": dep_result["resolved_dependencies"]
})
```

### Pattern 2: Parallel Domain Analysis

```python
# Example: Orchestrate parallel domain analysis across all domains

result = domain_orchestration_tool.execute({
    "domains": ["all"],  # All 20 ARIA domains
    "analysis_mode": "comprehensive",
    "parallel_limit": 20,
    "output_format": "json"
})

# Access cross-domain analysis
shared_deps = result["cross_domain_analysis"]["shared_dependencies"]
feature_flags = result["cross_domain_analysis"]["feature_flags_needed"]

# Get prioritized tasks
p0_tasks = result["aggregated_tasks"]["P0"]
p1_tasks = result["aggregated_tasks"]["P1"]
```

### Pattern 3: Configuration Update with Validation

```python
# Example: Validate → Propose → Apply → Verify

# Step 1: Validate current configuration
validate_result = configuration_manager_tool.execute({
    "action": "validate",
    "config_files": [
        {"path": "/path/to/flake.nix", "type": "flake.nix"},
        {"path": "/path/to/pixi.toml", "type": "pixi.toml"}
    ],
    "validation_rules": {
        "syntax": True,
        "schema": True,
        "dependencies": True,
        "security": True
    }
})

# Step 2: Propose update
propose_result = configuration_manager_tool.execute({
    "action": "propose_update",
    "update_spec": {
        "component": "qdrant",
        "version": "1.7.0",
        "target_file": "docker-compose.data.yml"
    }
})

# Step 3: Apply if no critical issues
if len(propose_result["results"][0]["errors"]) == 0:
    apply_result = configuration_manager_tool.execute({
        "action": "apply_update",
        "update_spec": propose_result["proposed_changes"]
    })
```

### Pattern 4: Workflow Verification Pipeline

```python
# Example: Syntax → Smoke Tests → Integration Tests → E2E

workflow_result = workflow_verifier_tool.execute({
    "verification_type": "comprehensive",
    "workflow_paths": [".github/workflows/*.yml"],
    "target_environment": "ci",
    "test_scope": {
        "domains": ["Agent Runtime", "Tool Execution"],
        "include_optional": False
    },
    "verification_options": {
        "timeout_seconds": 300,
        "parallel_execution": True,
        "fail_fast": False,
        "generate_report": True
    }
})

# Check overall status
if workflow_result["verification_summary"]["overall_status"] == "passed":
    print("All workflows verified successfully")
else:
    # Review failures
    for workflow in workflow_result["workflow_results"]:
        if workflow["status"] == "invalid":
            print(f"Failed: {workflow['workflow_name']}")
            print(workflow["syntax_validation"]["errors"])
```

---

## Schema Evolution

### Version Compatibility

```yaml
version_strategy:
  major: "Breaking changes (e.g., 1.x.x → 2.0.0)"
  minor: "New fields, backward compatible (e.g., 1.1.x → 1.2.0)"
  patch: "Bug fixes, clarifications (e.g., 1.1.1 → 1.1.2)"

compatibility:
  - Maintain old schema versions for 6 months
  - Provide migration guides for major versions
  - Support multi-version schemas during transition
```

### Change Log Template

```markdown
## [Version] - YYYY-MM-DD

### Added
- New fields: ...
- New enum values: ...

### Changed
- Modified field types: ...
- Updated descriptions: ...

### Deprecated
- Fields to be removed: ...
- Migration path: ...

### Removed
- Removed fields: ...
- Reason: ...

### Fixed
- Bug fixes: ...
```

---

## Integration Guidelines

### Adding a New Tool Schema

1. **Define the Tool**
   ```yaml
   tool_definition:
     name: "new_tool_name"
     purpose: "Clear, concise purpose"
     domain: "ARIA domain alignment"
   ```

2. **Create Schema File**
   - Filename: `{tool-name}.schema.json`
   - Follow naming convention: kebab-case
   - Include all required fields

3. **Define Input Schema**
   ```json
   {
     "inputSchema": {
       "type": "object",
       "properties": {
         // Define inputs
       },
       "required": ["..."]
     }
   }
   ```

4. **Define Output Schema**
   ```json
   {
     "outputSchema": {
       "type": "object",
       "properties": {
         // Define outputs
       },
       "required": ["..."]
     }
   }
   ```

5. **Validate**
   - Run JSON Schema validator
   - Test with sample data
   - Document in this INDEX.md

6. **Document**
   - Add to README.md
   - Provide usage examples
   - Specify integration points

---

## Testing

### Unit Tests

```python
# test_mcp_schemas.py
import json
import jsonschema
from pathlib import Path

def test_schema_validity():
    """Test that all schemas are valid JSON Schema"""
    schema_dir = Path("manifests/mcp/schemas")

    for schema_file in schema_dir.glob("*.schema.json"):
        with open(schema_file) as f:
            schema = json.load(f)

        # Validate against JSON Schema meta-schema
        jsonschema.Draft202012Validator.check_schema(schema)
        print(f"✅ {schema_file.name} is valid")

def test_required_fields():
    """Test that all schemas have required fields"""
    required = ["$schema", "$id", "title", "description", "version"]

    schema_dir = Path("manifests/mcp/schemas")

    for schema_file in schema_dir.glob("*.schema.json"):
        with open(schema_file) as f:
            schema = json.load(f)

        for field in required:
            assert field in schema, f"{schema_file.name} missing {field}"

        print(f"✅ {schema_file.name} has all required fields")
```

### Integration Tests

```python
# test_tool_execution.py
def test_repository_analysis_integration():
    """Test repository analysis tool end-to-end"""
    from aria_mcp import RepositoryAnalysisTool

    tool = RepositoryAnalysisTool()

    result = tool.execute({
        "repository_url": "https://github.com/agixt/agixt",
        "analysis_depth": "standard",
        "target_domain": "Agent Runtime"
    })

    # Validate output against schema
    assert "repository" in result
    assert "installation_method" in result
    assert result["installation_method"]["primary"] in ["nix", "pixi", "docker", "cargo"]
```

---

## Performance Benchmarks

### Target Metrics

```yaml
tool_performance:
  repository_analysis:
    latency_p50: "<2s"
    latency_p95: "<5s"
    latency_p99: "<10s"

  domain_orchestration:
    latency_p50: "<30s (20 domains)"
    latency_p95: "<60s"
    throughput: "1 analysis/min"

  configuration_manager:
    validation_latency: "<500ms"
    update_latency: "<2s"

  workflow_verifier:
    syntax_check: "<1s"
    smoke_tests: "<30s"
    integration_tests: "<5min"

  feature_flag_manager:
    list_latency: "<100ms"
    switch_latency: "<2s"

  dependency_resolver:
    graph_analysis: "<5s"
    resolution: "<10s"

  agent_capability_mapper:
    discovery: "<1s"
    mapping: "<2s"
    search: "<500ms"
```

---

## References

- **JSON Schema Specification**: https://json-schema.org/draft/2020-12/schema
- **MCP Protocol**: https://modelcontextprotocol.io
- **ARIA Architecture**: /home/user/ripple-env/MANUS_ARIA_ORCHESTRATOR.md
- **Capability Registry**: /home/user/ripple-env/manifests/capability-registry/

---

**Index Version**: 1.0.0
**Last Updated**: 2026-01-09
**Maintainer**: L8 Tool Execution MCP Team Lead
