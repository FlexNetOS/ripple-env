---
name: ai-assistants
description: AI-powered development tools configuration and usage
icon: 🤖
category: development
tools:
  - aichat
  - ai
  - ai-code
  - ai-explain
  - ai-review
---

# AI Assistants

This environment includes AI-powered development tools to enhance your ROS2 development workflow.

## aichat - Foundation AI CLI

**aichat** is the default AI assistant - a tiny, provider-agnostic CLI that works with multiple AI providers.

### Supported Providers

| Provider | Model Examples | API Key Env Var |
|----------|---------------|-----------------|
| Anthropic | claude-3-opus, claude-3-sonnet | `ANTHROPIC_API_KEY` |
| OpenAI | gpt-4, gpt-4-turbo, gpt-3.5-turbo | `OPENAI_API_KEY` |
| Google | gemini-pro, gemini-1.5-pro | `GOOGLE_API_KEY` |
| Ollama | llama2, codellama, mistral | (local, no key) |
| Azure OpenAI | gpt-4, gpt-35-turbo | `AZURE_OPENAI_API_KEY` |

### Quick Start

```bash
# Set your API key (choose your provider)
export ANTHROPIC_API_KEY="your-key-here"
# or
export OPENAI_API_KEY="your-key-here"

# Basic usage
ai "explain what ROS2 topics are"

# Code assistance
ai-code "write a ROS2 publisher node in Python"

# Code review
ai-review "review this launch file for best practices"

# Explain code
cat src/my_node.py | ai-explain
```

### Available Aliases

| Alias | Command | Purpose |
|-------|---------|---------|
| `ai` | `aichat` | General AI chat |
| `ai-code` | `aichat --role coder` | Code generation |
| `ai-explain` | `aichat --role explain` | Code explanation |
| `ai-review` | `aichat --role reviewer` | Code review |

### Configuration

aichat stores configuration in `~/.config/aichat/config.yaml`:

```yaml
# Example configuration
model: claude-3-sonnet-20240229
save: true
highlight: true
temperature: 0.7

# Custom roles
roles:
  - name: ros2-expert
    prompt: |
      You are a ROS2 expert. Help with:
      - Node development (Python/C++)
      - Launch files
      - Message/Service definitions
      - Best practices for robotics
```

### Using with Ollama (Local Models)

For offline/private AI assistance:

```bash
# Install Ollama (if not already)
curl -fsSL https://ollama.com/install.sh | sh

# Pull a coding model
ollama pull codellama

# Use with aichat
aichat --model ollama:codellama "write a ROS2 subscriber"
```

### ROS2-Specific Usage

```bash
# Explain a ROS2 concept
ai "explain ROS2 QoS profiles"

# Generate a launch file
ai-code "create a launch file that starts a camera node and image processor"

# Debug an error
ai "why am I getting 'could not find package' in colcon build"

# Review code
cat src/robot_controller/robot_controller/controller.py | ai-review
```

### Tips

1. **Pipe code for context**: `cat file.py | ai "explain this"`
2. **Use roles for consistency**: `ai-code` for generation, `ai-review` for feedback
3. **Save sessions**: Use `aichat -s session-name` to continue conversations
4. **Local models**: Use Ollama for private/offline work

## Future: Aider Integration

Phase 2 will add **Aider** for Git-integrated AI pair programming:

```bash
# Coming soon
aider src/my_package/  # AI-assisted editing with Git commits
```

## Environment Variables

Add to your shell profile or `.envrc`:

```bash
# Choose one provider
export ANTHROPIC_API_KEY="sk-ant-..."
export OPENAI_API_KEY="sk-..."
export GOOGLE_API_KEY="..."

# Optional: Set default model
export AICHAT_MODEL="claude-3-sonnet-20240229"
```

## Home-Manager Configuration

If using home-manager, enable aichat:

```nix
{
  programs.aichat = {
    enable = true;
    settings = {
      model = "claude-3-sonnet-20240229";
      save = true;
      highlight = true;
    };
  };
}
```
