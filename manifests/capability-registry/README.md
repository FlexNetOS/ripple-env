# Capability Registry

Agent capability registry for 10,498+ ready-to-use APIs.

## Overview

This registry catalogs APIs available for agent automation and task execution. The data is sourced from [API-mega-list](https://github.com/cporter202/API-mega-list).

## Registry Structure

```
capability-registry/
├── README.md           # This file
└── api-registry.json   # Main registry manifest
```

## Categories

| Category | APIs | Description |
|----------|------|-------------|
| Automation | 4,825 | Workflow and task automation |
| Lead Generation | 3,452 | Contact discovery and enrichment |
| Social Media | 3,268 | Platform data extraction |
| Developer Tools | 2,652 | Development and DevOps |
| Ecommerce | 2,440 | Product and marketplace |
| Other | 1,297 | Miscellaneous utilities |
| AI | 1,208 | ML and AI services |
| Videos | 979 | Video platform APIs |
| Integrations | 890 | Third-party connectivity |
| Real Estate | 851 | Property data |
| Jobs | 848 | Recruitment and listings |
| Open Source | 768 | Repository and project APIs |
| SEO Tools | 710 | Search optimization |
| Agents | 697 | Autonomous task execution |
| News | 590 | News aggregation |
| Travel | 397 | Booking and hospitality |
| MCP Servers | 131 | Model Context Protocol |
| Business | 2 | Enterprise operations |

## Usage

### Loading the Registry

```python
import json

with open('manifests/capability-registry/api-registry.json') as f:
    registry = json.load(f)

# Get all categories
categories = registry['categories']

# Find APIs by use case
ai_apis = [c for c in categories if 'ai' in c['id']]
```

### Agent Integration

The registry defines capability groups for agent orchestration:

- **dataCollection**: Website and platform scraping
- **leadGeneration**: Contact discovery
- **contentProcessing**: AI-powered content analysis
- **marketIntelligence**: Competitive insights
- **developerTooling**: DevOps integration
- **agentOrchestration**: Autonomous task execution

## Updating

To sync with the latest API-mega-list:

```bash
# Fetch latest category counts from source
curl -s https://raw.githubusercontent.com/cporter202/API-mega-list/main/README.md
```

## Source

- Repository: https://github.com/cporter202/API-mega-list
- Update frequency: Daily
- Total APIs: 10,498+
