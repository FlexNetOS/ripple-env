# AGiXT Prompt Templates

This directory contains prompt templates for AGiXT agent operations.

## ROS2 Prompts

### ros2-command-parser.txt
Parses natural language into structured robot commands.
- Input: Natural language command (e.g., "Move forward 2 meters")
- Output: JSON with command_type, target, speed, direction, params

### ros2-safety-check.txt
Validates robot commands for safety constraints.
- Input: Parsed command from ros2-command-parser
- Output: Safety assessment with potential modifications

## DevOps Prompts

### task-analysis.txt
Analyzes DevOps tasks and creates execution plans.
- Input: Task description
- Output: Structured analysis with steps, tools, risks

### script-generation.txt
Generates scripts based on task analysis.
- Input: Task analysis from task-analysis prompt
- Output: Scripts (bash, YAML, Nix, etc.) with documentation

### security-review.txt
Reviews generated scripts for security issues.
- Input: Generated scripts from script-generation prompt
- Output: Security assessment with recommendations

## Usage

Prompts are referenced in chain definitions. To use standalone:

```bash
# Via AGiXT API
curl -X POST http://localhost:7437/api/v1/prompt \
  -H "Content-Type: application/json" \
  -d '{
    "prompt_name": "ros2-command-parser",
    "prompt_category": "default",
    "prompt_content": "$(cat config/agixt/prompts/ros2-command-parser.txt)"
  }'
```

## Template Variables

Prompts use AGiXT template variables:
- `{user_input}`: User's input message
- `{chain_step_N_output}`: Output from step N in a chain
- `{context}`: Additional context if provided
- `{agent_name}`: Current agent name

## Creating New Prompts

1. Create a `.txt` file in this directory
2. Use clear instructions and examples
3. Specify expected input/output formats
4. Include template variables where needed
5. Reference in chain definitions or use via API
