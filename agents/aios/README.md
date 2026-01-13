# AIOS Sample Agents

This directory contains sample agents for the AIOS (AI Agent Operating System) platform.

## Available Agents

### ros2-coordinator
Coordinates ROS2 robot actions through natural language commands.

**Capabilities:**
- Parse natural language into robot commands
- Track robot state
- Execute movement, rotation, and navigation commands

**Usage:**
```bash
pixi run -e aios python -m cerebrum run agents/aios/ros2-coordinator
```

### devops-assistant
Assists with CI/CD, infrastructure, and automation tasks.

**Capabilities:**
- Analyze DevOps tasks
- Generate scripts (bash, YAML, Nix)
- Security review of configurations

**Usage:**
```bash
pixi run -e aios python -m cerebrum run agents/aios/devops-assistant
```

## Agent Structure

Each agent follows the Cerebrum SDK structure:

```
agent-name/
├── entry.py          # Main agent code with run() method
├── config.json       # Agent configuration and metadata
└── meta_requirements.txt  # Python dependencies
```

## Prerequisites

1. **Start AIOS Kernel:**
   ```bash
   aios install  # First time only
   aios start
   ```

2. **Start LocalAI (for LLM inference):**
   ```bash
   localai start
   ```

3. **Run an agent:**
   ```bash
   pixi run -e aios python -m cerebrum run agents/aios/<agent-name>
   ```

## Creating New Agents

1. Create a new directory in `agents/aios/`
2. Create `entry.py` with a class containing a `run(task: str) -> str` method
3. Create `config.json` with agent metadata
4. Create `meta_requirements.txt` with dependencies
5. Test with: `pixi run -e aios python -m cerebrum run agents/aios/<your-agent>`

## Integration with AGiXT

These AIOS agents can work alongside AGiXT:
- AIOS provides the agent runtime (scheduler, memory, tools)
- AGiXT provides orchestration and chain execution
- LocalAI provides local LLM inference

```
┌─────────────────────────────────────┐
│         Your Application            │
├─────────────────────────────────────┤
│  AGiXT (Port 7437) - Orchestration  │
├─────────────────────────────────────┤
│  AIOS Kernel (Port 8000) - Runtime  │
├─────────────────────────────────────┤
│  LocalAI (Port 8080) - Inference    │
└─────────────────────────────────────┘
```

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `AIOS_URL` | AIOS Kernel URL | http://localhost:8000 |
| `LOCALAI_URL` | LocalAI API URL | http://localhost:8080 |
| `AGIXT_URL` | AGiXT API URL | http://localhost:7437 |
