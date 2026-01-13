# AGiXT Chain Definitions

This directory contains chain definitions for AGiXT agent orchestration.

## Available Chains

### ros2-command-chain.json
Parses natural language commands for ROS2 robot control:
1. Parse command to JSON structure (command_type, target, speed, params)
2. Safety check for the parsed command

### devops-automation-chain.json
DevOps task automation pipeline:
1. Analyze the task requirements
2. Generate appropriate scripts/configurations
3. Security review of generated content

## Usage

Import chains via AGiXT API or UI:

```bash
# Via API
curl -X POST http://localhost:7437/api/v1/chain \
  -H "Content-Type: application/json" \
  -d @config/agixt/chains/ros2-command-chain.json

# Via agixt CLI
agixt up
# Then import via UI at http://localhost:3437
```

## Chain Structure

Each chain is a JSON file with:
- `chain_name`: Unique identifier
- `description`: Human-readable description
- `steps`: Array of execution steps

Each step contains:
- `step_number`: Order of execution
- `agent_name`: Agent to execute the step
- `prompt_type`: Type of prompt (Chain, Prompt, etc.)
- `prompt_name`: Name of the prompt template
- `step_config`: Model and generation settings

## Creating New Chains

1. Create a new JSON file in this directory
2. Define steps with appropriate prompts
3. Import via AGiXT API/UI
4. Test with sample inputs
