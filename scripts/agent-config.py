#!/usr/bin/env python3
"""
Agent Configuration Management Tool

Unified configuration validation, generation, and management for agents,
skills, commands, and models.

Usage:
    agent-config validate [--all | --agents | --skills | --models]
    agent-config generate agent <name> [--model=MODEL] [--category=CATEGORY]
    agent-config generate skill <name> [--category=CATEGORY]
    agent-config generate command <name>
    agent-config list [agents | skills | commands | models]
    agent-config export [--format=FORMAT]
    agent-config check-consistency
"""

import argparse
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

# Project paths
PROJECT_ROOT = Path(__file__).parent.parent
CLAUDE_DIR = PROJECT_ROOT / ".claude"
CONFIG_DIR = CLAUDE_DIR / "config"
AGENTS_DIR = CLAUDE_DIR / "agents"
SKILLS_DIR = CLAUDE_DIR / "skills"
COMMANDS_DIR = CLAUDE_DIR / "commands"


@dataclass
class ValidationResult:
    """Result of a validation check."""

    valid: bool
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


@dataclass
class AgentConfig:
    """Parsed agent configuration."""

    name: str
    role: str
    model: str = "sonnet"
    context: str = ""
    priority: str = "medium"
    trigger_keywords: list[str] = field(default_factory=list)
    responsibilities: list[str] = field(default_factory=list)
    handoffs: list[dict] = field(default_factory=list)


@dataclass
class SkillConfig:
    """Parsed skill configuration."""

    name: str
    description: str
    icon: str = ""
    category: str = "development"
    tools: list[str] = field(default_factory=list)


def load_json(path: Path) -> dict[str, Any]:
    """Load JSON file."""
    with open(path) as f:
        return json.load(f)


def load_yaml(path: Path) -> dict[str, Any]:
    """Load YAML file."""
    with open(path) as f:
        return yaml.safe_load(f)


def parse_frontmatter(content: str) -> tuple[dict[str, Any], str]:
    """Parse YAML frontmatter from markdown content."""
    if not content.startswith("---"):
        return {}, content

    parts = content.split("---", 2)
    if len(parts) < 3:
        return {}, content

    try:
        frontmatter = yaml.safe_load(parts[1])
        body = parts[2].strip()
        return frontmatter or {}, body
    except yaml.YAMLError:
        return {}, content


def parse_agent_file(path: Path) -> AgentConfig | None:
    """Parse an agent markdown file."""
    content = path.read_text()
    frontmatter, body = parse_frontmatter(content)

    if not frontmatter:
        # Try to extract from content
        name_match = re.search(r"^#\s+(.+?)\s*(?:Agent)?$", content, re.MULTILINE)
        name = name_match.group(1).lower().replace(" ", "-") if name_match else path.stem

        return AgentConfig(name=name, role=f"{name} Agent")

    return AgentConfig(
        name=frontmatter.get("name", path.stem),
        role=frontmatter.get("role", ""),
        model=frontmatter.get("model", "sonnet"),
        context=frontmatter.get("context", ""),
        priority=frontmatter.get("priority", "medium"),
    )


def parse_skill_file(path: Path) -> SkillConfig | None:
    """Parse a skill markdown file."""
    skill_file = path / "SKILL.md" if path.is_dir() else path
    if not skill_file.exists():
        return None

    content = skill_file.read_text()
    frontmatter, _ = parse_frontmatter(content)

    if not frontmatter:
        return None

    return SkillConfig(
        name=frontmatter.get("name", path.stem if path.is_file() else path.name),
        description=frontmatter.get("description", ""),
        icon=frontmatter.get("icon", ""),
        category=frontmatter.get("category", "development"),
        tools=frontmatter.get("tools", []),
    )


class ConfigValidator:
    """Validates agent configuration files."""

    VALID_MODELS = {
        "opus",
        "sonnet",
        "haiku",
        "gpt4",
        "gpt4o",
        "kimi-k2",
        "kimi-k2-thinking",
        "localai",
        "ollama",
        "vllm",
    }

    VALID_CATEGORIES = {
        "core",
        "domain",
        "analysis",
        "specialized",
        "robotics",
        "devops",
        "infrastructure",
        "security",
        "development",
    }

    VALID_PRIORITIES = {"highest", "high", "medium", "low"}

    def __init__(self):
        self.schema = self._load_schema()

    def _load_schema(self) -> dict[str, Any]:
        """Load the agent schema."""
        schema_path = CONFIG_DIR / "agent-schema.json"
        if schema_path.exists():
            return load_json(schema_path)
        return {}

    def validate_agent(self, agent: AgentConfig) -> ValidationResult:
        """Validate an agent configuration."""
        errors = []
        warnings = []

        # Name validation
        if not re.match(r"^[a-z0-9-]+(-agent)?$", agent.name):
            errors.append(f"Invalid agent name format: {agent.name}")

        # Model validation
        if agent.model and agent.model not in self.VALID_MODELS:
            errors.append(f"Invalid model: {agent.model}. Valid: {self.VALID_MODELS}")

        # Role validation
        if not agent.role or len(agent.role) < 5:
            errors.append("Agent role must be at least 5 characters")

        # Priority validation
        if agent.priority and agent.priority not in self.VALID_PRIORITIES:
            warnings.append(f"Non-standard priority: {agent.priority}")

        return ValidationResult(valid=len(errors) == 0, errors=errors, warnings=warnings)

    def validate_skill(self, skill: SkillConfig) -> ValidationResult:
        """Validate a skill configuration."""
        errors = []
        warnings = []

        # Name validation
        if not re.match(r"^[a-z0-9-]+$", skill.name):
            errors.append(f"Invalid skill name format: {skill.name}")

        # Description validation
        if not skill.description or len(skill.description) < 10:
            errors.append("Skill description must be at least 10 characters")

        # Category validation
        if skill.category and skill.category not in self.VALID_CATEGORIES:
            warnings.append(f"Non-standard category: {skill.category}")

        # Tools validation
        if not skill.tools:
            warnings.append("Skill has no tools defined")

        return ValidationResult(valid=len(errors) == 0, errors=errors, warnings=warnings)

    def validate_models_config(self, config: dict) -> ValidationResult:
        """Validate models.json configuration."""
        errors = []
        warnings = []

        if "models" not in config:
            errors.append("Missing 'models' section")
            return ValidationResult(valid=False, errors=errors)

        # Check required model categories
        required_categories = {"claude"}
        missing = required_categories - set(config["models"].keys())
        if missing:
            errors.append(f"Missing model categories: {missing}")

        # Check routing configuration
        if "routing" in config:
            routing = config["routing"]
            if "default" not in routing:
                warnings.append("No default model specified in routing")

        return ValidationResult(valid=len(errors) == 0, errors=errors, warnings=warnings)

    def validate_settings(self, settings: dict) -> ValidationResult:
        """Validate settings.json configuration."""
        errors = []
        warnings = []

        # Check permissions
        if "permissions" not in settings:
            warnings.append("No permissions defined")
        else:
            perms = settings["permissions"]
            if "allow" not in perms and "deny" not in perms:
                errors.append("Permissions must have 'allow' or 'deny' lists")

        # Check context
        if "context" in settings:
            ctx = settings["context"]
            if "include" in ctx:
                for path in ctx["include"]:
                    full_path = PROJECT_ROOT / path
                    if not full_path.exists():
                        warnings.append(f"Context include path not found: {path}")

        return ValidationResult(valid=len(errors) == 0, errors=errors, warnings=warnings)

    def check_consistency(self) -> ValidationResult:
        """Check cross-file consistency."""
        errors = []
        warnings = []

        # Load all configurations
        agents = self._load_all_agents()
        skills = self._load_all_skills()
        models_config = self._load_models_config()
        coordinator = self._load_coordinator()

        # Check: All agents in coordinator are defined
        if coordinator:
            defined_agents = {a.name for a in agents}
            # Parse coordinator for referenced agents
            coord_content = (AGENTS_DIR / "coordinator.md").read_text()
            referenced = set(re.findall(r"\|\s*([a-z-]+-agent)\s*\|", coord_content))
            missing = referenced - defined_agents
            if missing:
                errors.append(f"Coordinator references undefined agents: {missing}")

        # Check: Skills referenced by agents exist
        skill_names = {s.name for s in skills}
        for agent in agents:
            agent_path = AGENTS_DIR / f"{agent.name}.md"
            if agent_path.exists():
                content = agent_path.read_text()
                skill_refs = re.findall(r"skills/([a-z0-9-]+)/", content)
                missing_skills = set(skill_refs) - skill_names
                if missing_skills:
                    warnings.append(f"Agent {agent.name} references missing skills: {missing_skills}")

        # Check: Model references are valid
        for agent in agents:
            if agent.model and agent.model not in self.VALID_MODELS:
                errors.append(f"Agent {agent.name} uses invalid model: {agent.model}")

        return ValidationResult(valid=len(errors) == 0, errors=errors, warnings=warnings)

    def _load_all_agents(self) -> list[AgentConfig]:
        """Load all agent configurations."""
        agents = []
        if AGENTS_DIR.exists():
            for path in AGENTS_DIR.glob("*.md"):
                agent = parse_agent_file(path)
                if agent:
                    agents.append(agent)
        return agents

    def _load_all_skills(self) -> list[SkillConfig]:
        """Load all skill configurations."""
        skills = []
        if SKILLS_DIR.exists():
            for path in SKILLS_DIR.iterdir():
                if path.is_dir():
                    skill = parse_skill_file(path)
                    if skill:
                        skills.append(skill)
        return skills

    def _load_models_config(self) -> dict:
        """Load models configuration."""
        models_path = CONFIG_DIR / "models.json"
        if models_path.exists():
            return load_json(models_path)
        return {}

    def _load_coordinator(self) -> AgentConfig | None:
        """Load coordinator agent."""
        coord_path = AGENTS_DIR / "coordinator.md"
        if coord_path.exists():
            return parse_agent_file(coord_path)
        return None


class ConfigGenerator:
    """Generates new agent configuration files."""

    AGENT_TEMPLATE = '''# {title}

This file configures Claude Code's behavior for {domain} tasks.

---
name: {name}
role: {role}
model: {model}
context: {context}
priority: {priority}
---

## Identity

You are the {title}, specialized in {specialization}.

## Core Responsibilities

1. **{resp1}** - {resp1_desc}
2. **{resp2}** - {resp2_desc}
3. **{resp3}** - {resp3_desc}

## Decision Rules

### When to Act
- {rule1}
- {rule2}

### When to Delegate
- {delegate1}
- {delegate2}

## Available Commands

| Command | Purpose |
|---------|---------|
| `{cmd1}` | {cmd1_desc} |
| `{cmd2}` | {cmd2_desc} |

## Context Loading

When working on {domain} tasks, load:
- `.claude/skills/{skill}/SKILL.md`

## Handoff Rules

- **To Coordinator**: When task is complete or needs different expertise
- **From Coordinator**: When {domain} expertise is requested
'''

    SKILL_TEMPLATE = '''---
name: {name}
description: {description}
icon: {icon}
category: {category}
tools:
{tools_yaml}
---

# {title}

## Overview

{overview}

## Prerequisites

- Active development shell (`nom develop` or `direnv allow`)

## Commands

### {section1_title}
```bash
{section1_cmd}
```

## Best Practices

1. {practice1}
2. {practice2}

## Troubleshooting

### Common Issues
- {issue1}
- {issue2}

## Related Skills

- [Related Skill](../related/SKILL.md) - Description
'''

    COMMAND_TEMPLATE = '''---
name: {name}
description: {description}
---

# /{name}

{instructions}

## Usage

```
/{name} [options]
```

## Options

- `--option1`: Description

## Examples

```bash
/{name}
/{name} --option1 value
```
'''

    def generate_agent(
        self,
        name: str,
        model: str = "sonnet",
        category: str = "domain",
    ) -> str:
        """Generate a new agent configuration."""
        title = name.replace("-", " ").title()
        if not name.endswith("-agent"):
            name = f"{name}-agent"

        return self.AGENT_TEMPLATE.format(
            name=name,
            title=f"{title} Agent",
            role=f"{title} Specialist",
            model=model,
            context=category,
            priority="medium",
            domain=category,
            specialization=f"{title.lower()} operations",
            resp1="Primary Task",
            resp1_desc="Main responsibility description",
            resp2="Secondary Task",
            resp2_desc="Secondary responsibility description",
            resp3="Support Task",
            resp3_desc="Support responsibility description",
            rule1="Condition for taking action",
            rule2="Another condition for action",
            delegate1="When to hand off to another agent",
            delegate2="Another handoff condition",
            cmd1="command1",
            cmd1_desc="Command description",
            cmd2="command2",
            cmd2_desc="Command description",
            skill=name.replace("-agent", ""),
        )

    def generate_skill(
        self,
        name: str,
        category: str = "development",
    ) -> str:
        """Generate a new skill configuration."""
        title = name.replace("-", " ").title()
        tools_yaml = "  - tool1\n  - tool2"

        return self.SKILL_TEMPLATE.format(
            name=name,
            title=f"{title} Skills",
            description=f"{title} development and operations",
            icon="🔧",
            category=category,
            tools_yaml=tools_yaml,
            overview=f"This skill provides expertise in {title.lower()} operations.",
            section1_title="Basic Commands",
            section1_cmd=f"# {title} command example",
            practice1="Best practice 1",
            practice2="Best practice 2",
            issue1="Common issue and solution",
            issue2="Another common issue",
        )

    def generate_command(self, name: str) -> str:
        """Generate a new command configuration."""
        title = name.replace("-", " ").title()

        return self.COMMAND_TEMPLATE.format(
            name=name,
            description=f"{title} command for agent operations",
            instructions=f"Execute {title.lower()} operations.",
        )


class ConfigExporter:
    """Exports unified configuration."""

    def export_unified_config(self) -> dict[str, Any]:
        """Export all configurations as a unified structure."""
        validator = ConfigValidator()

        config = {
            "$schema": "./agent-schema.json",
            "version": "1.0.0",
            "name": "ripple-env",
            "description": "ROS2 Humble development environment agent configuration",
            "agents": self._export_agents(validator),
            "skills": self._export_skills(validator),
            "commands": self._export_commands(),
            "models": self._export_models(),
        }

        return config

    def _export_agents(self, validator: ConfigValidator) -> dict:
        """Export agent configurations."""
        agents = validator._load_all_agents()
        return {
            "coordinator": "coordinator",
            "default": "coordinator",
            "enabled": [a.name for a in agents],
            "definitions": [
                {
                    "name": a.name,
                    "role": a.role,
                    "model": a.model,
                    "context": a.context,
                    "priority": a.priority,
                }
                for a in agents
            ],
        }

    def _export_skills(self, validator: ConfigValidator) -> dict:
        """Export skill configurations."""
        skills = validator._load_all_skills()
        return {
            "enabled": [s.name for s in skills],
            "definitions": [
                {
                    "name": s.name,
                    "description": s.description,
                    "icon": s.icon,
                    "category": s.category,
                    "tools": s.tools,
                }
                for s in skills
            ],
        }

    def _export_commands(self) -> dict:
        """Export command configurations."""
        commands = []
        if COMMANDS_DIR.exists():
            for path in COMMANDS_DIR.glob("*.md"):
                content = path.read_text()
                frontmatter, _ = parse_frontmatter(content)
                if frontmatter:
                    commands.append(
                        {
                            "name": frontmatter.get("name", path.stem),
                            "description": frontmatter.get("description", ""),
                        }
                    )
        return {
            "enabled": [c["name"] for c in commands],
            "definitions": commands,
        }

    def _export_models(self) -> dict:
        """Export model configurations."""
        models_path = CONFIG_DIR / "models.json"
        if models_path.exists():
            config = load_json(models_path)
            return {
                "default": config.get("routing", {}).get("default", "sonnet"),
                "definitions": config.get("models", {}),
                "routing": [
                    {"taskType": k, "model": v}
                    for k, v in config.get("routing", {}).get("by_task", {}).items()
                ],
            }
        return {"default": "sonnet", "definitions": {}, "routing": []}


def cmd_validate(args: argparse.Namespace) -> int:
    """Run validation commands."""
    validator = ConfigValidator()
    all_valid = True

    if args.target in ("all", "agents"):
        print("\n=== Validating Agents ===")
        agents = validator._load_all_agents()
        for agent in agents:
            result = validator.validate_agent(agent)
            status = "✓" if result.valid else "✗"
            print(f"  {status} {agent.name}")
            for err in result.errors:
                print(f"      ERROR: {err}")
            for warn in result.warnings:
                print(f"      WARN: {warn}")
            all_valid = all_valid and result.valid

    if args.target in ("all", "skills"):
        print("\n=== Validating Skills ===")
        skills = validator._load_all_skills()
        for skill in skills:
            result = validator.validate_skill(skill)
            status = "✓" if result.valid else "✗"
            print(f"  {status} {skill.name}")
            for err in result.errors:
                print(f"      ERROR: {err}")
            for warn in result.warnings:
                print(f"      WARN: {warn}")
            all_valid = all_valid and result.valid

    if args.target in ("all", "models"):
        print("\n=== Validating Models Config ===")
        models_config = validator._load_models_config()
        if models_config:
            result = validator.validate_models_config(models_config)
            status = "✓" if result.valid else "✗"
            print(f"  {status} models.json")
            for err in result.errors:
                print(f"      ERROR: {err}")
            for warn in result.warnings:
                print(f"      WARN: {warn}")
            all_valid = all_valid and result.valid

    if args.target == "all":
        print("\n=== Checking Consistency ===")
        result = validator.check_consistency()
        status = "✓" if result.valid else "✗"
        print(f"  {status} Cross-file consistency")
        for err in result.errors:
            print(f"      ERROR: {err}")
        for warn in result.warnings:
            print(f"      WARN: {warn}")
        all_valid = all_valid and result.valid

    print()
    return 0 if all_valid else 1


def cmd_generate(args: argparse.Namespace) -> int:
    """Run generation commands."""
    generator = ConfigGenerator()

    if args.type == "agent":
        content = generator.generate_agent(args.name, args.model, args.category)
        name = args.name if args.name.endswith("-agent") else f"{args.name}-agent"
        output_path = AGENTS_DIR / f"{name}.md"
    elif args.type == "skill":
        content = generator.generate_skill(args.name, args.category)
        output_dir = SKILLS_DIR / args.name
        output_dir.mkdir(exist_ok=True)
        output_path = output_dir / "SKILL.md"
    elif args.type == "command":
        content = generator.generate_command(args.name)
        output_path = COMMANDS_DIR / f"{args.name}.md"
    else:
        print(f"Unknown type: {args.type}")
        return 1

    if output_path.exists() and not args.force:
        print(f"File already exists: {output_path}")
        print("Use --force to overwrite")
        return 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(content)
    print(f"Generated: {output_path}")
    return 0


def cmd_list(args: argparse.Namespace) -> int:
    """List configurations."""
    validator = ConfigValidator()

    if args.type in ("all", "agents"):
        print("\n=== Agents ===")
        agents = validator._load_all_agents()
        for agent in sorted(agents, key=lambda a: a.name):
            print(f"  {agent.name:<30} model={agent.model:<10} {agent.role}")

    if args.type in ("all", "skills"):
        print("\n=== Skills ===")
        skills = validator._load_all_skills()
        for skill in sorted(skills, key=lambda s: s.name):
            print(f"  {skill.icon} {skill.name:<25} [{skill.category}] {skill.description[:50]}")

    if args.type in ("all", "commands"):
        print("\n=== Commands ===")
        if COMMANDS_DIR.exists():
            for path in sorted(COMMANDS_DIR.glob("*.md")):
                content = path.read_text()
                frontmatter, _ = parse_frontmatter(content)
                desc = frontmatter.get("description", "") if frontmatter else ""
                print(f"  /{path.stem:<25} {desc[:50]}")

    if args.type in ("all", "models"):
        print("\n=== Models ===")
        models_config = validator._load_models_config()
        for category, models in models_config.get("models", {}).items():
            print(f"  [{category}]")
            for name, config in models.items():
                use_case = config.get("use_case", "")
                print(f"    {name:<15} {use_case}")

    print()
    return 0


def cmd_export(args: argparse.Namespace) -> int:
    """Export unified configuration."""
    exporter = ConfigExporter()
    config = exporter.export_unified_config()

    if args.format == "json":
        output = json.dumps(config, indent=2)
    elif args.format == "yaml":
        output = yaml.dump(config, default_flow_style=False, sort_keys=False)
    else:
        print(f"Unknown format: {args.format}")
        return 1

    if args.output:
        Path(args.output).write_text(output)
        print(f"Exported to: {args.output}")
    else:
        print(output)

    return 0


def cmd_check_consistency(args: argparse.Namespace) -> int:
    """Check cross-file consistency."""
    validator = ConfigValidator()
    result = validator.check_consistency()

    print("\n=== Consistency Check ===")
    if result.valid:
        print("  ✓ All consistency checks passed")
    else:
        print("  ✗ Consistency issues found:")
        for err in result.errors:
            print(f"      ERROR: {err}")

    for warn in result.warnings:
        print(f"      WARN: {warn}")

    print()
    return 0 if result.valid else 1


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Agent Configuration Management Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # validate command
    validate_parser = subparsers.add_parser("validate", help="Validate configurations")
    validate_parser.add_argument(
        "target",
        nargs="?",
        default="all",
        choices=["all", "agents", "skills", "models"],
        help="What to validate",
    )

    # generate command
    generate_parser = subparsers.add_parser("generate", help="Generate new configurations")
    generate_parser.add_argument("type", choices=["agent", "skill", "command"])
    generate_parser.add_argument("name", help="Name for the new configuration")
    generate_parser.add_argument("--model", default="sonnet", help="Model for agents")
    generate_parser.add_argument("--category", default="development", help="Category")
    generate_parser.add_argument("--force", action="store_true", help="Overwrite existing")

    # list command
    list_parser = subparsers.add_parser("list", help="List configurations")
    list_parser.add_argument(
        "type",
        nargs="?",
        default="all",
        choices=["all", "agents", "skills", "commands", "models"],
    )

    # export command
    export_parser = subparsers.add_parser("export", help="Export unified configuration")
    export_parser.add_argument("--format", default="json", choices=["json", "yaml"])
    export_parser.add_argument("--output", "-o", help="Output file path")

    # check-consistency command
    subparsers.add_parser("check-consistency", help="Check cross-file consistency")

    args = parser.parse_args()

    commands = {
        "validate": cmd_validate,
        "generate": cmd_generate,
        "list": cmd_list,
        "export": cmd_export,
        "check-consistency": cmd_check_consistency,
    }

    return commands[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
