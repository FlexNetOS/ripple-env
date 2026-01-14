# On-Call and Escalation Procedures

**Status:** Template (Team-Specific Configuration Required)
**Last Updated:** 2026-01-14
**Purpose:** Define escalation paths, contact information, and response procedures for production incidents

---

## ⚠️ Configuration Required

This document provides a template for on-call procedures. **You must customize it with your team-specific information:**

- Contact names and information
- Communication channel details (Slack, email, phone)
- Escalation matrix based on your organization
- Response time expectations aligned with SLAs

**Security Note:** Consider storing contact information in a secure location (encrypted secrets, password manager) rather than directly in this file, especially for production systems.

---

## Quick Reference

| Severity | Response Time | Primary Contact | Escalation Path |
|----------|---------------|-----------------|-----------------|
| **P0 - Critical** | Immediate (15 min) | [Name/Role] | L1 → L2 → L3 → Management |
| **P1 - High** | <1 hour | [Name/Role] | L1 → L2 → L3 |
| **P2 - Medium** | <4 hours | [Name/Role] | L1 → L2 |
| **P3 - Low** | <24 hours | [Name/Role] | L1 |

---

## Severity Definitions

### P0 - Critical (Production Down)

**Definition**: Complete service outage affecting all users or data loss in progress

**Examples**:
- All services unreachable
- Database corruption or data loss
- Security breach or active attack
- Critical infrastructure failure (Vault, Keycloak)

**Response**:
- Immediate page/call to on-call engineer
- Notification to incident commander
- War room/bridge call initiated within 15 minutes
- Management notification immediate

**Resolution Target**: 1 hour

---

### P1 - High (Major Impact)

**Definition**: Significant functionality impaired, subset of users affected, or security concern

**Examples**:
- Critical service degraded (high latency, errors)
- Authentication failures (Keycloak/Vault issues)
- API errors affecting 10%+ of requests
- Certificate expiration imminent (<24 hours)
- Data inconsistency detected

**Response**:
- Page to on-call engineer
- Notification to team lead within 1 hour
- Status update every 2 hours
- Incident report required

**Resolution Target**: 4 hours

---

### P2 - Medium (Minor Impact)

**Definition**: Non-critical functionality impaired, workaround available, or isolated issue

**Examples**:
- Single service unhealthy but redundancy covers
- Performance degradation under high load
- Non-critical API endpoint errors
- Monitoring gaps or alerting issues
- Certificate expiration within 7 days

**Response**:
- Message to on-call engineer (async acceptable)
- Status update at next standup
- Incident report optional

**Resolution Target**: 1 business day

---

### P3 - Low (Minimal Impact)

**Definition**: Cosmetic issues, minor bugs, or feature requests

**Examples**:
- Documentation errors
- UI/UX improvements
- Non-urgent dependency updates
- Feature enhancement requests

**Response**:
- Ticket created in backlog
- Triaged at next planning session
- No immediate action required

**Resolution Target**: Next sprint

---

## Escalation Matrix

### Level 1 (L1) - On-Call Engineer

**Responsibilities**:
- Initial triage and assessment
- Execute runbooks and emergency procedures
- Gather logs and diagnostic information
- Escalate if unable to resolve within SLA

**Contact**: [Name/Handle]
- Primary: [Email/Slack/Phone]
- Backup: [Email/Slack/Phone]
- Hours: 24/7 rotation (weekly)

**Tools Access**:
- Production environment (read/write)
- Monitoring dashboards (Grafana)
- Logging systems (Loki/Prometheus)
- Incident management system

---

### Level 2 (L2) - Senior Engineer / Team Lead

**Responsibilities**:
- Technical escalation for complex issues
- Authorization for risky operations (database rollback, force restart)
- Coordinate with other teams if cross-service issue
- Decision authority for temporary fixes vs proper resolution

**Contact**: [Name/Handle]
- Primary: [Email/Slack/Phone]
- Backup: [Email/Slack/Phone]
- Hours: Business hours + on-call escalation

**Authority**:
- Approve emergency changes
- Authorize maintenance windows
- Engage external vendors

---

### Level 3 (L3) - Engineering Manager / Architect

**Responsibilities**:
- Executive decision-making for major incidents
- Resource allocation (call in additional engineers)
- Customer/stakeholder communication
- Post-incident review coordination

**Contact**: [Name/Handle]
- Primary: [Email/Slack/Phone]
- Hours: Reachable 24/7 for P0/P1

**Authority**:
- Authorize system downtime
- Approve major architectural changes under duress
- External communication

---

### Level 4 (L4) - Executive / Management

**Responsibilities**:
- Business impact decisions
- Customer communication (C-level)
- Legal/compliance notification
- Press/public relations

**Contact**: [Name/Title]
- Primary: [Email/Phone]
- Hours: P0 only, via L3 escalation

---

## Communication Channels

### Primary Incident Channel

**Slack**: `#incidents` (or create dedicated channel per incident)
- Use for real-time coordination
- Pin key information (runbook links, status updates)
- Set channel topic with current status

**Bridge Call**: [Conference Line/Zoom Link]
- For P0/P1 incidents requiring voice coordination
- Record for post-incident review

**Incident Management**: [Tool Name: Jira/PagerDuty/StatusPage]
- Create ticket immediately for P0/P1
- Update status every 30 minutes (P0) or 2 hours (P1)
- Close with RCA (root cause analysis)

---

### Status Page

**External**: [URL]
- Update for P0/P1 customer-facing issues
- Template messages available in runbooks

**Internal**: [URL/Slack Channel]
- Update for all incidents affecting internal users

---

## Response Procedures

### P0 - Critical Incident Response

**Within 5 minutes**:
1. Acknowledge page/alert
2. Join incident bridge call
3. Assign incident commander (on-call or senior engineer)
4. Create incident channel: `#incident-YYYYMMDD-description`

**Within 15 minutes**:
5. Initial assessment and severity confirmation
6. Execute relevant emergency runbook (see `docs/cookbooks/EMERGENCY_PROCEDURES.md`)
7. Page L2 if root cause unclear
8. Update status page if customer-facing

**Within 30 minutes**:
9. Status update in incident channel
10. Notify management if not resolved
11. Consider escalation to L3

**Resolution**:
12. Confirm service restored
13. Monitor for 30 minutes to ensure stability
14. Schedule post-incident review within 48 hours
15. Update status page: "Resolved"

---

### Runbook Execution Priority

1. **Check monitoring**: Grafana dashboards, Prometheus alerts
2. **Review logs**: Use `docs/TROUBLESHOOTING.md` for common patterns
3. **Execute emergency procedures**: `docs/cookbooks/EMERGENCY_PROCEDURES.md`
4. **Validate services**: `./scripts/validate-e2e.sh`
5. **If stuck**: Escalate immediately (don't delay)

---

## Contact Information

### Engineering Team

| Name | Role | Slack | Email | Phone | On-Call Week |
|------|------|-------|-------|-------|--------------|
| [TBD] | Lead Engineer | @handle | email@ | +1-xxx-xxx-xxxx | Week 1 |
| [TBD] | Senior Engineer | @handle | email@ | +1-xxx-xxx-xxxx | Week 2 |
| [TBD] | DevOps Engineer | @handle | email@ | +1-xxx-xxx-xxxx | Week 3 |
| [TBD] | SRE | @handle | email@ | +1-xxx-xxx-xxxx | Week 4 |

**Rotation Schedule**: [Link to PagerDuty/OpsGenie schedule]

---

### External Vendors

| Vendor | Service | Contact | Support Hours | SLA |
|--------|---------|---------|---------------|-----|
| [Cloud Provider] | Infrastructure | support@provider | 24/7 | 1 hour |
| [Monitoring SaaS] | Observability | support@monitor | 24/7 | 4 hours |
| [Security Vendor] | Secrets/Auth | support@security | Business hours | Next day |

---

### Internal Teams

| Team | Contact | Slack | Purpose |
|------|---------|-------|---------|
| Platform | [Name] | #platform | Infrastructure, Kubernetes |
| Security | [Name] | #security | Incidents, vulnerabilities |
| Database | [Name] | #database | PostgreSQL, Neo4j |
| Network | [Name] | #network | Connectivity, DNS, load balancers |

---

## Post-Incident Procedures

### Required Within 48 Hours

1. **Post-Incident Review (PIR)** meeting
   - Attendees: Incident commander, responders, team lead
   - Duration: 1 hour
   - Outcome: Action items and timeline

2. **Root Cause Analysis (RCA)** document
   - Template: `docs/templates/RCA_TEMPLATE.md` (create if needed)
   - Sections: Timeline, root cause, contributing factors, action items

3. **Documentation Updates**
   - Update runbooks with lessons learned
   - Add new troubleshooting patterns to `docs/TROUBLESHOOTING.md`
   - Update monitoring/alerting if gaps identified

4. **Customer Communication** (if applicable)
   - Incident summary to affected customers
   - Preventive measures being taken
   - Compensation/credits if SLA violated

---

## Tools and Access

### Required Access for On-Call

- [ ] Production environment (via VPN if remote)
- [ ] Grafana dashboards (read-only minimum)
- [ ] Prometheus (query access)
- [ ] Vault (break-glass credentials for emergency)
- [ ] GitHub (repository access for runbooks)
- [ ] Docker host (SSH access)
- [ ] Incident management system (PagerDuty/Jira)
- [ ] Status page admin (if customer-facing)

### Emergency Credentials

**Location**: [Password manager / Secrets vault]
**Break-glass procedure**:
1. Authenticate with MFA
2. Retrieve emergency credentials
3. Log usage in audit trail
4. Rotate credentials after incident resolved

---

## Training and Onboarding

### New On-Call Engineer Checklist

- [ ] Review this document
- [ ] Read all runbooks: `docs/RUNBOOKS.md` and `docs/cookbooks/EMERGENCY_PROCEDURES.md`
- [ ] Walkthrough of monitoring dashboards
- [ ] Test access to all production systems
- [ ] Shadow experienced on-call for one week
- [ ] Participate in fire drill/game day

### Fire Drill Schedule

**Frequency**: Quarterly
**Scenarios**:
- Database failover
- Service outage simulation
- Security incident response
- Certificate expiration

---

## Related Documentation

- [Emergency Procedures](cookbooks/EMERGENCY_PROCEDURES.md) - Immediate incident response
- [Runbooks](RUNBOOKS.md) - Operational procedures
- [Troubleshooting](TROUBLESHOOTING.md) - Common issues and solutions
- [Security Guide](SECURITY_GUIDE.md) - Security incident response
- [Observability](OBSERVABILITY-QUICK-START.md) - Monitoring and alerting
- [Backup & Restore](cookbooks/BACKUP_RESTORE.md) - Disaster recovery

---

## Appendix: Communication Templates

### P0 Initial Notification (Slack)
```
@here P0 INCIDENT
Summary: [Brief description]
Impact: [Services/users affected]
Incident Channel: #incident-YYYYMMDD-description
Bridge: [Conference link]
Commander: @username
ETA: Investigating, update in 15 minutes
```

### Status Page Update
```
We are currently investigating an issue affecting [service/feature].
Our team is working to resolve this as quickly as possible.
Next update: [Time]
```

### Resolution Notification
```
The incident affecting [service] has been resolved.
Root cause: [Brief explanation]
Preventive measures: [Actions being taken]
Post-incident review: [Date/time]
We apologize for any inconvenience.
```

---

**Remember**: When in doubt, escalate early. False escalations are better than missed critical incidents.
