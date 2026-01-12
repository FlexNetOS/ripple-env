# Bytebase Database CI/CD Setup Guide

## Overview

Bytebase is an enterprise-grade database DevOps and CI/CD platform that enables teams to manage database schema changes with the same rigor as application code. This guide covers the setup, configuration, and usage of Bytebase in the agentic systems infrastructure.

**Status**: P3-002 Implementation
**Version**: 2.13.0
**Last Updated**: 2026-01-10

## Table of Contents

1. [Quick Start](#quick-start)
2. [Architecture](#architecture)
3. [Installation](#installation)
4. [Configuration](#configuration)
5. [First-Time Setup](#first-time-setup)
6. [Database Connection](#database-connection)
7. [Workflow Management](#workflow-management)
8. [Change Management](#change-management)
9. [Git Integration](#git-integration)
10. [Backup & Recovery](#backup--recovery)
11. [Monitoring](#monitoring)
12. [Troubleshooting](#troubleshooting)
13. [Best Practices](#best-practices)

---

## Quick Start

### Prerequisites

- Docker and Docker Compose installed
- Docker network created: `agentic-network`
- PostgreSQL database accessible

### Start Bytebase

```bash
# Create Docker network if not exists
docker network create agentic-network 2>/dev/null || true

# Start Bytebase and its PostgreSQL database
docker compose -f docker/docker-compose.bytebase.yml up -d

# Check status
docker compose -f docker/docker-compose.bytebase.yml ps
```

### Access Bytebase

1. Open browser: http://localhost:5678
2. Default credentials:
   - Email: `admin@bytebase.com`
   - Password: `changeme`
3. Complete initial setup wizard

**IMPORTANT**: Change the default password immediately in production!

---

## Architecture

### Components

```
┌─────────────────────────────────────────────────────────────┐
│                    Bytebase Container                       │
│  - Web UI (Port 5678)                                       │
│  - API Server                                                │
│  - Change Management Engine                                  │
│  - Risk Analysis Engine                                      │
│  - Backup/Restore Manager                                    │
└────────────┬───────────────────────────────────┬────────────┘
             │                                   │
    ┌────────▼─────────┐            ┌──────────▼──────────┐
    │ PostgreSQL       │            │  Target Databases   │
    │ (Bytebase Meta)  │            │  (Dev/Staging/Prod) │
    │ - Config Storage │            │                     │
    │ - Change History │            │  - PostgreSQL       │
    │ - User/Audit Log │            │  - MySQL            │
    └──────────────────┘            │  - MongoDB, etc.    │
                                    └─────────────────────┘
```

### Environments

| Environment | Approval | Risk Level | Backups | Use Case |
|---|---|---|---|---|
| **Development** | Optional | Low | 7 days | Active development |
| **Staging** | Required (1) | Medium | 30 days | Pre-production testing |
| **Production** | Required (2+) | High | 90 days | Live systems |

---

## Installation

### Step 1: Verify Prerequisites

```bash
# Check Docker is running
docker --version

# Check network exists
docker network ls | grep agentic-network

# Check PostgreSQL is accessible (if not in compose)
psql -h localhost -U postgres -d postgres -c "SELECT version();"
```

### Step 2: Deploy Bytebase

```bash
# Navigate to project root
cd /home/user/ripple-env

# Start services
docker compose -f docker/docker-compose.bytebase.yml up -d

# Wait for health checks
sleep 30

# Verify containers are healthy
docker compose -f docker/docker-compose.bytebase.yml ps

# Check logs
docker compose -f docker/docker-compose.bytebase.yml logs -f bytebase
```

### Step 3: Initialize Database

Bytebase will automatically initialize its PostgreSQL database with:
- Schema for storing configuration
- User management tables
- Audit log tables
- Change request history

No manual initialization required.

---

## Configuration

### Environment Variables

Edit `docker-compose.bytebase.yml` to customize:

```yaml
environment:
  # PostgreSQL connection string
  PG_URL: postgresql://user:pass@host:5432/bytebase

  # Port configuration
  BB_PORT: "5678"

  # External URL for webhooks and integrations
  BB_EXTERNAL_URL: http://bytebase.example.com:5678

  # Data directory
  # BYTEBASE_DATA_DIR: /var/opt/bytebase

  # Debug mode (true/false)
  # BB_DEBUG: "false"
```

### Project Configuration

Located at: `/config/bytebase/project.yaml`

Key sections:

1. **Environments** - Define dev/staging/prod with approval requirements
2. **Policies** - SQL review rules, DDL/DML restrictions
3. **Risk Management** - High-risk statement detection
4. **Backup Settings** - Retention and scheduling
5. **Git Integration** - GitOps configuration
6. **Access Control** - RBAC definitions

Apply configuration after first startup (see First-Time Setup).

---

## First-Time Setup

### 1. Access Web UI

```
http://localhost:5678
```

### 2. Login with Default Credentials

- Email: `admin@bytebase.com`
- Password: `changeme`

### 3. Complete Setup Wizard

The wizard guides you through:
- Setting admin password (REQUIRED)
- Creating workspaces
- Configuring instance URLs
- Setting up environments

### 4. Add Database Instances

Navigate to: **Admin > Database Instances**

#### Development Database

```
Engine:     PostgreSQL
Host:       postgres (or Docker service name)
Port:       5432
Database:   agentic_dev
Username:   postgres
Password:   (from secrets)
SSL:        Not Required
Environment: Development
```

Click "Test Connection" to verify.

#### Staging Database (Optional)

```
Engine:     PostgreSQL
Host:       staging-db.example.com
Port:       5432
Database:   agentic_staging
Username:   postgres
Password:   (from secrets)
SSL:        Required
Environment: Staging
```

#### Production Database (Optional)

```
Engine:     PostgreSQL
Host:       prod-db.example.com
Port:       5432
Database:   agentic_prod
Username:   postgres
Password:   (from secrets)
SSL:        Required (TLS 1.2+)
Environment: Production
```

### 5. Create Project

Navigate to: **Projects > Create Project**

```
Project Name:     Agentic Systems
Project Key:      AS
Description:      Database CI/CD for agentic infrastructure
Environment:      (select all three)
Visibility:       Private
```

### 6. Import Databases

After project creation, add databases:
1. Click "Add Database"
2. Select database instance
3. Configure backup schedule
4. Enable change notifications

### 7. Configure Approval Policies

Navigate to: **Project Settings > Change Approval**

```
Development:   No approval required
Staging:       1 approval required (DBA)
Production:    2+ approvals required (DBA + Owner)
```

---

## Database Connection

### Connection String Format

```
postgresql://username:password@host:port/database

Examples:
- postgresql://postgres:changeme@localhost:5432/agentic_dev
- postgresql://postgres:secret@staging-db:5432/agentic_staging
- postgresql://postgres:secret@prod-db:5432/agentic_prod
```

### Testing Connection

```bash
# From command line
psql "postgresql://postgres:changeme@localhost:5432/agentic_dev" \
  -c "SELECT version();"

# Or via Bytebase Web UI
# Admin > Database Instances > [Instance] > Test Connection
```

### Connection Pooling

For high-concurrency environments, configure pooling:

```yaml
# In docker-compose.bytebase.yml or DB instance settings
environment:
  PG_MAX_CONNECTIONS: "500"
  PG_SHARED_BUFFERS: "256MB"
  PG_EFFECTIVE_CACHE_SIZE: "1GB"
```

---

## Workflow Management

### Change Request Lifecycle

```
1. Create Change Request
   ↓
2. Add SQL statements
   ↓
3. Risk analysis & review
   ↓
4. Approval workflow (env-dependent)
   ↓
5. Schedule deployment
   ↓
6. Execute migration
   ↓
7. Verify & rollback (if needed)
   ↓
8. Close issue
```

### Creating a Change Request

#### Via Web UI

1. **Projects > [Project] > Create Issue**
2. **Select Issue Type**: "Change Database Schema"
3. **Configure Change**:
   - Title: e.g., "Add user preferences table"
   - Description: Business context
   - Environment: Select target (dev/staging/prod)
4. **Add SQL Statements**:
   ```sql
   CREATE TABLE user_preferences (
     id SERIAL PRIMARY KEY,
     user_id INTEGER NOT NULL REFERENCES users(id),
     theme VARCHAR(50),
     created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
   );
   CREATE INDEX idx_user_preferences_user_id
     ON user_preferences(user_id);
   ```
5. **Risk Analysis**: Review detected risks
6. **Submit for Review**

#### Via API

```bash
curl -X POST http://localhost:5678/api/v1/projects/AS/issues \
  -H "Authorization: Bearer <token>" \
  -H "Content-Type: application/json" \
  -d '{
    "title": "Add user_preferences table",
    "description": "Store user UI preferences",
    "type": "DATABASE_CHANGE",
    "assignee": "dba@example.com",
    "statement": "CREATE TABLE user_preferences ..."
  }'
```

### Approval Workflow

1. **Development**: Auto-approve or manual
2. **Staging**: 1 approval (DBA/Database Owner)
3. **Production**: 2+ approvals (DBA + Project Owner)

Approve via:
- Web UI: **Issues > [Issue] > Approve**
- Email notification (if configured)

### Deployment

After approval:

1. **Schedule Deployment**:
   - Immediate (low-risk changes)
   - Specific date/time (maintenance window)
   - Manual execution

2. **Monitor Execution**:
   - Real-time SQL execution logs
   - Transaction status
   - Performance metrics

3. **Verify Success**:
   - Schema validation
   - Data integrity checks
   - Application testing

---

## Change Management

### SQL Review Rules

Configure automatic SQL validation in project settings.

#### Built-in Rules

| Rule | Severity | Example |
|------|----------|---------|
| PRIMARY_KEY_REQUIRED | Error | Table must have PK |
| NO_NULL_IN_PK | Error | PK cols can't be NULL |
| FOREIGN_KEY_INDEX | Warning | FK should be indexed |
| NAMING_CONVENTION | Warning | Use snake_case |
| TEXT_TYPE_LIMIT | Warning | Limit TEXT fields |

#### Custom Rules

Add custom rules via **Project Settings > SQL Review**:

```yaml
rules:
  - name: no_truncate_in_prod
    enabled: true
    severity: error
    scope: PRODUCTION
    patterns:
      - "TRUNCATE TABLE.*"
    message: "TRUNCATE not allowed in production"

  - name: require_soft_delete
    enabled: true
    severity: warning
    scope: ALL
    patterns:
      - "DELETE FROM.*"
    message: "Consider soft delete (update deleted_at)"
```

### Risk Analysis

Bytebase automatically detects and alerts on:

- **High Risk**:
  - DROP TABLE/DATABASE
  - ALTER ... DROP COLUMN
  - DELETE without WHERE clause
  - Schema changes in production

- **Medium Risk**:
  - ALTER TABLE ... MODIFY
  - CREATE INDEX (performance impact)
  - Large table migrations

- **Low Risk**:
  - Schema queries
  - Adding NOT NULL columns with defaults
  - Index creation on small tables

### Rollback Procedures

#### Automatic Rollback

Bytebase maintains schema before/after states. To rollback:

1. **Web UI**: **Issues > [Completed Issue] > Rollback**
2. Creates new change request with reverse SQL
3. Follows same approval workflow

#### Manual Rollback

```bash
# Connect to affected database
psql "postgresql://postgres:pass@host:5432/db" << EOF
  -- Previous schema state
  DROP TABLE IF EXISTS user_preferences;
  -- Or restore from backup
EOF
```

---

## Git Integration

### Enable GitOps (Optional)

Configure in project settings to sync schema with Git repository.

#### GitHub Integration

1. **Create GitHub Token**:
   - Settings > Developer settings > Personal access tokens
   - Scopes: `repo`, `workflow`

2. **Configure in Bytebase**:
   - **Project Settings > Git Sync**
   - Provider: GitHub
   - Repository: `owner/repo`
   - Branch: `main`
   - SQL directory: `schema/migrations/`

3. **Workflow**:
   - Changes committed to Git
   - Bytebase detects changes
   - Creates pull request or auto-applies
   - Tracks in Bytebase for audit

#### GitLab Integration

Similar process with GitLab tokens and repository configuration.

### File Structure

```
schema/
├── migrations/
│   ├── 001_initial_schema.sql
│   ├── 002_add_user_preferences.sql
│   └── 003_create_audit_log.sql
└── README.md
```

---

## Backup & Recovery

### Automatic Backups

Configured per environment:

```yaml
Development:  Daily, 7-day retention
Staging:      Daily, 30-day retention
Production:   Daily, 90-day retention
```

### Manual Backup

```bash
# Via Web UI
# Database > [Instance] > Backup > Create Backup

# Via API
curl -X POST http://localhost:5678/api/v1/databases/agentic_prod/backups \
  -H "Authorization: Bearer <token>"
```

### Point-in-Time Recovery (PITR)

Restore to specific timestamp:

```bash
# Web UI
# Database > [Instance] > Backup > Restore to Point in Time
# Select timestamp: 2026-01-10 14:30:00 UTC
```

### Backup Storage

Backups stored in:
- Docker volume: `bytebase-data` (local)
- S3 (configured in advanced settings)
- Custom locations (via config)

---

## Monitoring

### Health Checks

```bash
# Check Bytebase service
curl http://localhost:5678/health

# Check database connection
docker compose -f docker/docker-compose.bytebase.yml exec \
  bytebase curl http://localhost:5678/health

# View logs
docker compose -f docker/docker-compose.bytebase.yml logs bytebase
```

### Metrics

Monitor in Bytebase dashboard:

1. **Change Metrics**:
   - Issues created/completed
   - Deployment frequency
   - Approval cycle time

2. **Database Metrics**:
   - Table count
   - Data volume
   - Query performance

3. **Audit Metrics**:
   - User activity
   - Change history
   - Approval tracking

### Alerts

Configure notifications for:
- Change request created
- Approval required
- Deployment started/completed
- Errors or failures

**Configuration**: Project Settings > Notifications

---

## Troubleshooting

### Bytebase Won't Start

```bash
# Check logs
docker compose -f docker/docker-compose.bytebase.yml logs bytebase

# Common issues:
# 1. Port 5678 already in use
docker ps | grep 5678
# Solution: Change port in docker-compose.yml

# 2. PostgreSQL not accessible
docker compose -f docker/docker-compose.bytebase.yml exec postgres \
  pg_isready -U postgres
# Solution: Check PG_URL in environment

# 3. Volume permission denied
docker volume ls
docker volume inspect bytebase-data
# Solution: Check directory permissions
```

### Cannot Connect to Database

```bash
# Verify connection string
echo $PG_URL

# Test from Bytebase container
docker compose -f docker/docker-compose.bytebase.yml exec bytebase \
  psql "$PG_URL" -c "SELECT version();"

# Check credentials
docker compose -f docker/docker-compose.bytebase.yml exec postgres \
  psql -U postgres -d bytebase -c "SELECT count(*) FROM pg_user;"
```

### Deployment Fails

1. **Check SQL Syntax**: Review SQL in change request
2. **Check Locks**: Any long-running queries?
3. **Check Disk Space**: Sufficient space for schema changes?
4. **Review Logs**: Deployment error details

```bash
# View detailed logs
docker compose -f docker/docker-compose.bytebase.yml logs --tail 100 bytebase

# Check database logs
docker compose -f docker/docker-compose.bytebase.yml exec postgres \
  tail -f /var/log/postgresql.log
```

### Performance Issues

If Bytebase UI is slow:

```bash
# Check resource usage
docker stats bytebase

# Increase resource limits in docker-compose.yml
deploy:
  resources:
    limits:
      memory: 4gb
      cpus: '2.0'

# Restart
docker compose -f docker/docker-compose.bytebase.yml restart bytebase
```

---

## Best Practices

### 1. Change Management

- **Small, frequent changes** over large batches
- **Meaningful descriptions** for all change requests
- **Test in development** before staging
- **Schedule production changes** during maintenance windows
- **Review SQL carefully** for performance implications

### 2. Code Review

- **Require approval** for all production changes
- **Use code review tools** (GitHub PRs with Bytebase)
- **Document rationale** in change descriptions
- **Track business context** (tickets/requirements)

### 3. Testing

- **Validate changes** in development first
- **Performance test** before production
- **Smoke test** after deployment
- **Have rollback plan** for every change

### 4. Security

- **Change default password** immediately
- **Enable SSL** for database connections
- **Restrict database access** by role
- **Audit all changes** for compliance
- **Encrypt backups** for sensitive data

### 5. Disaster Recovery

- **Test backups** regularly
- **Document recovery procedures**
- **Maintain offsite backups** (S3, etc.)
- **Document RTO/RPO** for each database
- **Practice drills** with recovery procedures

### 6. Documentation

- **Document all policies** (approval, risk, etc.)
- **Keep schema documentation** updated
- **Maintain runbooks** for common tasks
- **Record change history** for compliance
- **Share knowledge** with team

### 7. Integration

- **Automate deployments** via CI/CD
- **Integrate with monitoring** (alerts)
- **Connect to ticketing** (Jira, etc.)
- **Enable webhooks** for notifications
- **Version control** schema changes (Git)

---

## Resources

### Official Documentation

- [Bytebase Documentation](https://www.bytebase.com/docs)
- [SQL Review Rules](https://www.bytebase.com/docs/sql-review)
- [Change Management](https://www.bytebase.com/docs/change-database)
- [API Reference](https://www.bytebase.com/docs/api)

### Related Components

- **Docker Compose**: `/docker/docker-compose.bytebase.yml`
- **Project Config**: `/config/bytebase/project.yaml`
- **Database Services**: `/docker/docker-compose.data.yml`

### Support

- [Community Slack](https://slack.bytebase.com)
- [GitHub Issues](https://github.com/bytebase/bytebase)
- [Email Support](mailto:support@bytebase.com)

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-01-10 | Initial Bytebase setup guide |

---

**Last Updated**: 2026-01-10
**Author**: ROS2 Humble Environment Team
**Status**: Stable
