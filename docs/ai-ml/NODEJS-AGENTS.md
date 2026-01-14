# Node.js Agent Frameworks for ROS2 Humble

This document describes the Node.js agent framework integration for the ROS2 Humble environment, including setup, usage, and best practices.

## Overview

The Node.js agent framework provides a foundation for deploying AI agents in ROS2 environments using:

- **Anthropic SDK**: For accessing Claude AI models
- **Model Context Protocol (MCP)**: For standardized agent communication
- **JavaScript/Node.js**: For rapid development and integration

## Architecture

```
┌─────────────────────────────────────────┐
│   ROS2 Humble Environment               │
│  ┌─────────────────────────────────────┐│
│  │   Node.js Agent Framework           ││
│  │  ┌─────────────────────────────────┐││
│  │  │  Anthropic Claude Integration   │││
│  │  └─────────────────────────────────┘││
│  │  ┌─────────────────────────────────┐││
│  │  │  Model Context Protocol (MCP)   │││
│  │  └─────────────────────────────────┘││
│  │  ┌─────────────────────────────────┐││
│  │  │  ROS2 Humble Interfaces         │││
│  │  └─────────────────────────────────┘││
│  └─────────────────────────────────────┘│
└─────────────────────────────────────────┘
```

## Installation

### Prerequisites

- Node.js 18.0 or higher
- npm or yarn
- Valid Anthropic API key

### Setup Steps

1. **Install dependencies**:
   ```bash
   cd /home/user/ripple-env
   npm install
   ```

2. **Configure API key**:
   ```bash
   export ANTHROPIC_API_KEY="your-api-key-here"
   ```

3. **Verify installation**:
   ```bash
   npm run agents:start
   ```

## Project Structure

```
/home/user/ripple-env/
├── package.json                 # Node.js project configuration
├── agents/
│   ├── index.js                 # Main agent framework
│   ├── ros2-integration.js       # ROS2-specific integrations (future)
│   └── mcp-handlers.js           # MCP protocol handlers (future)
└── docs/
    └── NODEJS-AGENTS.md         # This documentation
```

## Core Components

### ROS2Agent Class

The primary interface for agent operations.

```javascript
const { ROS2Agent } = require('./agents/index.js');

// Create an agent instance
const agent = new ROS2Agent();

// Process a query
const response = await agent.processQuery('What is ROS2?');

// View conversation history
const history = agent.getHistory();

// Clear conversation
agent.clearHistory();
```

### Agent Configuration

Customize agent behavior through configuration:

```javascript
const customConfig = {
  model: 'claude-opus-4-5-20251101',
  maxTokens: 8192,
  systemPrompt: 'Custom system instructions...',
};

const agent = new ROS2Agent(customConfig);
```

## Usage Examples

### Basic Query Processing

```javascript
const { ROS2Agent } = require('./agents/index.js');

async function example() {
  const agent = new ROS2Agent();

  const response = await agent.processQuery(
    'How do I launch a ROS2 node?'
  );

  console.log(response);
}

example().catch(console.error);
```

### Multi-turn Conversation

```javascript
async function conversation() {
  const agent = new ROS2Agent();

  // First turn
  await agent.processQuery('What is a ROS2 topic?');

  // Second turn (context preserved)
  await agent.processQuery('How do I subscribe to one?');

  // View full conversation
  console.log(agent.getHistory());
}
```

### Integration with ROS2 Nodes

For integration with actual ROS2 nodes, refer to the ROS2 Python/C++ bridge patterns:

```javascript
// Future: ROS2 service client
// const rosClient = new ROS2ServiceClient('service_name');
// const result = await rosClient.call(request);
```

## Running the Framework

### Start the Agent Service

```bash
npm run agents:start
```

This executes the example workflow demonstrating:
1. Agent initialization
2. Multi-turn conversation
3. Conversation history management

### Running as a Module

Use in your own scripts:

```javascript
const { ROS2Agent, agentConfig } = require('./agents/index.js');

// Your implementation
```

## Environment Variables

| Variable | Purpose | Required |
|----------|---------|----------|
| `ANTHROPIC_API_KEY` | Anthropic API authentication token | Yes |
| `NODE_ENV` | Environment (development/production) | No |
| `LOG_LEVEL` | Logging verbosity | No |

## Dependencies

### @anthropic-ai/sdk (^0.30.0)

Official Anthropic SDK for JavaScript/Node.js.

- Documentation: https://github.com/anthropics/anthropic-sdk-js
- Features: Streaming, vision, tools, batch processing

### @modelcontextprotocol/sdk (^1.0.0)

Model Context Protocol implementation for standardized agent communication.

- Specification: https://modelcontextprotocol.io
- Features: Server/client communication, resource management, tool calling

## Advanced Features

### Streaming Responses (Future)

```javascript
// Not yet implemented - planned for v2.0
const stream = await agent.processQueryStream('query');
stream.on('text', (chunk) => console.log(chunk));
```

### Tool Calling (Future)

```javascript
// Not yet implemented - planned for v2.0
agent.registerTool({
  name: 'ros2_launch',
  description: 'Launch a ROS2 package',
  execute: async (params) => { /* ... */ },
});
```

### MCP Integration (Future)

```javascript
// Not yet implemented - planned for v2.0
const mcp = new MCPServer(agent);
await mcp.start();
```

## Error Handling

The framework includes error handling for:

- Missing API keys
- Network/API failures
- Invalid queries
- Token limit exceeded

```javascript
try {
  const response = await agent.processQuery(query);
} catch (error) {
  console.error('Agent error:', error.message);
  // Handle specific error types as needed
}
```

## Performance Considerations

### Conversation History

- Conversation history grows with each message
- Consider clearing history for long-running sessions
- Use `agent.clearHistory()` to reset

### Token Usage

- Monitor token consumption with large contexts
- Adjust `maxTokens` based on requirements
- Consider implementing conversation summarization

### Latency

- API calls to Anthropic (~1-5 seconds typical)
- Local processing is minimal
- Consider batching for high-throughput applications

## Testing

### Unit Tests (Future)

```bash
npm test
```

### Manual Testing

```bash
npm run agents:start
```

### Integration Testing

Connect to actual ROS2 environment for full validation.

## Troubleshooting

### API Key Not Found

```
Error: ANTHROPIC_API_KEY environment variable not set
```

**Solution**: Set the environment variable
```bash
export ANTHROPIC_API_KEY="your-key-here"
```

### Module Not Found

```
Error: Cannot find module '@anthropic-ai/sdk'
```

**Solution**: Install dependencies
```bash
npm install
```

### Network Errors

Connection issues to Anthropic API indicate network/firewall problems.

**Solutions**:
- Check internet connectivity
- Verify API endpoint accessibility
- Check API key validity

## Best Practices

1. **Always set ANTHROPIC_API_KEY** before running agents
2. **Clear conversation history** periodically to manage memory
3. **Use appropriate max_tokens** for your use case
4. **Implement error handling** in production code
5. **Monitor API usage** and costs
6. **Version lock dependencies** in production
7. **Test locally** before deploying to ROS2 environment

## Next Steps

1. Explore the example workflow in `agents/index.js`
2. Create custom agents for specific ROS2 tasks
3. Integrate with ROS2 nodes and services
4. Extend with custom tools and capabilities

## Contributing

To extend the framework:

1. Create new files in `agents/` directory
2. Export classes and functions for reuse
3. Update this documentation
4. Test thoroughly with ROS2 environment

## References

- [Anthropic Documentation](https://docs.anthropic.com)
- [Claude API Guide](https://docs.anthropic.com/claude/reference/getting-started)
- [Model Context Protocol](https://modelcontextprotocol.io)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

## Support

For issues or questions:
- Check this documentation
- Review example code in `agents/index.js`
- Consult Anthropic API documentation
- Check ROS2 community resources

---

**Version**: 1.0.0
**Last Updated**: 2026-01-10
**Maintainer**: ROS2 Humble Environment Team
