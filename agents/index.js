#!/usr/bin/env node

/**
 * ROS2 Humble Agent Framework
 *
 * This module provides a foundation for deploying AI agents
 * in ROS2 environments using Node.js with Anthropic SDK
 * and Model Context Protocol support.
 */

const Anthropic = require('@anthropic-ai/sdk');

// Initialize Anthropic client
const client = new Anthropic({
  apiKey: process.env.ANTHROPIC_API_KEY,
});

/**
 * Agent configuration for ROS2 operations
 */
const agentConfig = {
  model: 'claude-opus-4-5-20251101',
  maxTokens: 4096,
  systemPrompt: `You are an AI agent for ROS2 Humble environments.
  You can help with:
  - ROS2 package management
  - Sensor and actuator control
  - Navigation and motion planning
  - Simulation and testing
  - Diagnostics and troubleshooting`,
};

/**
 * Agent class for ROS2 interactions
 */
class ROS2Agent {
  constructor(config = agentConfig) {
    this.config = config;
    this.conversationHistory = [];
  }

  /**
   * Process a user query through the agent
   * @param {string} userMessage - The user's message/query
   * @returns {Promise<string>} - The agent's response
   */
  async processQuery(userMessage) {
    try {
      // Add user message to conversation history
      this.conversationHistory.push({
        role: 'user',
        content: userMessage,
      });

      // Call Claude API with conversation history
      const response = await client.messages.create({
        model: this.config.model,
        max_tokens: this.config.maxTokens,
        system: this.config.systemPrompt,
        messages: this.conversationHistory,
      });

      // Extract assistant response
      const assistantMessage = response.content[0].text;

      // Add assistant response to history
      this.conversationHistory.push({
        role: 'assistant',
        content: assistantMessage,
      });

      return assistantMessage;
    } catch (error) {
      console.error('Error processing query:', error);
      throw error;
    }
  }

  /**
   * Clear conversation history
   */
  clearHistory() {
    this.conversationHistory = [];
  }

  /**
   * Get conversation history
   * @returns {Array} - The conversation history
   */
  getHistory() {
    return this.conversationHistory;
  }
}

/**
 * Main agent workflow example
 */
async function main() {
  console.log('ROS2 Humble Agent Framework Starting...\n');

  // Check for API key
  if (!process.env.ANTHROPIC_API_KEY) {
    console.error(
      'Error: ANTHROPIC_API_KEY environment variable not set.\n' +
      'Please set your API key: export ANTHROPIC_API_KEY="your-key-here"'
    );
    process.exit(1);
  }

  // Initialize agent
  const agent = new ROS2Agent();

  // Example workflow
  const workflows = [
    'What are the main components of ROS2 Humble?',
    'How do I create a simple ROS2 package?',
    'What is the Model Context Protocol and how does it integrate with agents?',
  ];

  console.log('Running example workflows:\n');

  for (const query of workflows) {
    console.log(`[User] ${query}`);
    try {
      const response = await agent.processQuery(query);
      console.log(`[Agent] ${response}\n`);
      console.log('-'.repeat(80) + '\n');
    } catch (error) {
      console.error(`Failed to process query: ${error.message}`);
    }
  }

  // Export for module usage
  console.log('Agent initialized and ready for use.');
  console.log('Conversation history entries:', agent.getHistory().length);
}

// Module exports
module.exports = {
  ROS2Agent,
  agentConfig,
};

// Run main workflow if script is executed directly
if (require.main === module) {
  main().catch(console.error);
}
