/**
 * AI Integration Service
 * Provides unified interface for LocalAI and AGiXT integrations
 */

// Configuration types
export interface LocalAIConfig {
  baseUrl: string;
  apiKey?: string;
  defaultModel?: string;
  timeout?: number;
}

export interface AGiXTConfig {
  baseUrl: string;
  apiKey?: string;
  agentName?: string;
  sandboxEnabled?: boolean;
}

export interface AIMessage {
  role: 'system' | 'user' | 'assistant';
  content: string;
}

export interface AICompletionOptions {
  model?: string;
  temperature?: number;
  maxTokens?: number;
  topP?: number;
  stream?: boolean;
}

export interface AICompletionResponse {
  id: string;
  content: string;
  model: string;
  usage?: {
    promptTokens: number;
    completionTokens: number;
    totalTokens: number;
  };
}

export interface AGiXTCommand {
  command: string;
  args?: Record<string, unknown>;
  conversationId?: string;
}

export interface AGiXTResponse {
  response: string;
  conversationId?: string;
  commands?: {
    name: string;
    result: unknown;
  }[];
}

// Default configurations
const DEFAULT_LOCALAI_CONFIG: LocalAIConfig = {
  baseUrl: 'http://localhost:8080',
  defaultModel: 'llama-3.2-3b-instruct',
  timeout: 60000,
};

const DEFAULT_AGIXT_CONFIG: AGiXTConfig = {
  baseUrl: 'http://localhost:7437',
  agentName: 'ripple-assistant',
  sandboxEnabled: true,
};

/**
 * LocalAI Integration Class
 */
export class LocalAIService {
  private config: LocalAIConfig;

  constructor(config?: Partial<LocalAIConfig>) {
    this.config = { ...DEFAULT_LOCALAI_CONFIG, ...config };
  }

  /**
   * Check if LocalAI service is available
   */
  async healthCheck(): Promise<boolean> {
    try {
      const response = await fetch(`${this.config.baseUrl}/v1/models`, {
        method: 'GET',
        headers: this.getHeaders(),
        signal: AbortSignal.timeout(5000),
      });
      return response.ok;
    } catch {
      return false;
    }
  }

  /**
   * Get available models
   */
  async getModels(): Promise<string[]> {
    try {
      const response = await fetch(`${this.config.baseUrl}/v1/models`, {
        method: 'GET',
        headers: this.getHeaders(),
      });

      if (!response.ok) {
        throw new Error(`Failed to fetch models: ${response.statusText}`);
      }

      const data = await response.json();
      return data.data?.map((m: { id: string }) => m.id) || [];
    } catch (error) {
      console.error('LocalAI: Failed to get models', error);
      return [];
    }
  }

  /**
   * Create a chat completion
   */
  async chatCompletion(
    messages: AIMessage[],
    options?: AICompletionOptions
  ): Promise<AICompletionResponse> {
    const response = await fetch(`${this.config.baseUrl}/v1/chat/completions`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({
        model: options?.model || this.config.defaultModel,
        messages,
        temperature: options?.temperature ?? 0.7,
        max_tokens: options?.maxTokens ?? 2048,
        top_p: options?.topP ?? 0.9,
        stream: options?.stream ?? false,
      }),
    });

    if (!response.ok) {
      throw new Error(`LocalAI completion failed: ${response.statusText}`);
    }

    const data = await response.json();
    return {
      id: data.id,
      content: data.choices?.[0]?.message?.content || '',
      model: data.model,
      usage: data.usage ? {
        promptTokens: data.usage.prompt_tokens,
        completionTokens: data.usage.completion_tokens,
        totalTokens: data.usage.total_tokens,
      } : undefined,
    };
  }

  /**
   * Generate embeddings
   */
  async createEmbeddings(texts: string[], model?: string): Promise<number[][]> {
    const response = await fetch(`${this.config.baseUrl}/v1/embeddings`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({
        model: model || 'embeddings',
        input: texts,
      }),
    });

    if (!response.ok) {
      throw new Error(`LocalAI embeddings failed: ${response.statusText}`);
    }

    const data = await response.json();
    return data.data?.map((d: { embedding: number[] }) => d.embedding) || [];
  }

  /**
   * Stream chat completion
   */
  async *streamChatCompletion(
    messages: AIMessage[],
    options?: Omit<AICompletionOptions, 'stream'>
  ): AsyncGenerator<string> {
    const response = await fetch(`${this.config.baseUrl}/v1/chat/completions`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({
        model: options?.model || this.config.defaultModel,
        messages,
        temperature: options?.temperature ?? 0.7,
        max_tokens: options?.maxTokens ?? 2048,
        top_p: options?.topP ?? 0.9,
        stream: true,
      }),
    });

    if (!response.ok || !response.body) {
      throw new Error(`LocalAI stream failed: ${response.statusText}`);
    }

    const reader = response.body.getReader();
    const decoder = new TextDecoder();

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;

      const chunk = decoder.decode(value);
      const lines = chunk.split('\n').filter(line => line.startsWith('data: '));

      for (const line of lines) {
        const data = line.slice(6);
        if (data === '[DONE]') return;

        try {
          const parsed = JSON.parse(data);
          const content = parsed.choices?.[0]?.delta?.content;
          if (content) yield content;
        } catch {
          // Skip invalid JSON
        }
      }
    }
  }

  private getHeaders(): HeadersInit {
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
    };
    if (this.config.apiKey) {
      headers['Authorization'] = `Bearer ${this.config.apiKey}`;
    }
    return headers;
  }
}

/**
 * AGiXT Integration Class
 */
export class AGiXTService {
  private config: AGiXTConfig;

  constructor(config?: Partial<AGiXTConfig>) {
    this.config = { ...DEFAULT_AGIXT_CONFIG, ...config };
  }

  /**
   * Check if AGiXT service is available
   */
  async healthCheck(): Promise<boolean> {
    try {
      const response = await fetch(`${this.config.baseUrl}/api/status`, {
        method: 'GET',
        headers: this.getHeaders(),
        signal: AbortSignal.timeout(5000),
      });
      return response.ok;
    } catch {
      return false;
    }
  }

  /**
   * Get available agents
   */
  async getAgents(): Promise<string[]> {
    try {
      const response = await fetch(`${this.config.baseUrl}/api/agent`, {
        method: 'GET',
        headers: this.getHeaders(),
      });

      if (!response.ok) {
        throw new Error(`Failed to fetch agents: ${response.statusText}`);
      }

      const data = await response.json();
      return data.agents || [];
    } catch (error) {
      console.error('AGiXT: Failed to get agents', error);
      return [];
    }
  }

  /**
   * Get available chains
   */
  async getChains(): Promise<string[]> {
    try {
      const response = await fetch(`${this.config.baseUrl}/api/chain`, {
        method: 'GET',
        headers: this.getHeaders(),
      });

      if (!response.ok) {
        throw new Error(`Failed to fetch chains: ${response.statusText}`);
      }

      const data = await response.json();
      return data.chains || [];
    } catch (error) {
      console.error('AGiXT: Failed to get chains', error);
      return [];
    }
  }

  /**
   * Chat with an agent
   */
  async chat(
    message: string,
    conversationId?: string,
    agentName?: string
  ): Promise<AGiXTResponse> {
    const response = await fetch(
      `${this.config.baseUrl}/api/agent/${agentName || this.config.agentName}/chat`,
      {
        method: 'POST',
        headers: this.getHeaders(),
        body: JSON.stringify({
          user_input: message,
          conversation_id: conversationId || 'default',
          injected_memories: 1,
        }),
      }
    );

    if (!response.ok) {
      throw new Error(`AGiXT chat failed: ${response.statusText}`);
    }

    const data = await response.json();
    return {
      response: data.response || '',
      conversationId: data.conversation_id,
      commands: data.commands,
    };
  }

  /**
   * Execute a command
   */
  async executeCommand(command: AGiXTCommand): Promise<unknown> {
    const response = await fetch(
      `${this.config.baseUrl}/api/agent/${this.config.agentName}/command`,
      {
        method: 'POST',
        headers: this.getHeaders(),
        body: JSON.stringify({
          command_name: command.command,
          command_args: command.args || {},
          conversation_id: command.conversationId || 'default',
        }),
      }
    );

    if (!response.ok) {
      throw new Error(`AGiXT command failed: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Run a chain
   */
  async runChain(
    chainName: string,
    userInput: string,
    agentName?: string,
    conversationId?: string
  ): Promise<AGiXTResponse> {
    const response = await fetch(`${this.config.baseUrl}/api/chain/${chainName}/run`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({
        agent_name: agentName || this.config.agentName,
        user_input: userInput,
        conversation_id: conversationId || 'default',
      }),
    });

    if (!response.ok) {
      throw new Error(`AGiXT chain failed: ${response.statusText}`);
    }

    const data = await response.json();
    return {
      response: data.response || '',
      conversationId: data.conversation_id,
    };
  }

  /**
   * Execute code in sandbox
   */
  async executeInSandbox(
    code: string,
    language: 'python' | 'node' | 'bash' | 'rust' | 'go' = 'python'
  ): Promise<{ output: string; exitCode: number }> {
    if (!this.config.sandboxEnabled) {
      throw new Error('Sandbox execution is disabled');
    }

    const response = await fetch(`${this.config.baseUrl}/api/sandbox/execute`, {
      method: 'POST',
      headers: this.getHeaders(),
      body: JSON.stringify({
        code,
        language,
        timeout: 60,
      }),
    });

    if (!response.ok) {
      throw new Error(`AGiXT sandbox execution failed: ${response.statusText}`);
    }

    const data = await response.json();
    return {
      output: data.output || '',
      exitCode: data.exit_code ?? -1,
    };
  }

  /**
   * Get conversation history
   */
  async getConversationHistory(
    conversationId: string,
    agentName?: string,
    limit?: number
  ): Promise<AIMessage[]> {
    const response = await fetch(
      `${this.config.baseUrl}/api/conversation/${agentName || this.config.agentName}/${conversationId}?limit=${limit || 50}`,
      {
        method: 'GET',
        headers: this.getHeaders(),
      }
    );

    if (!response.ok) {
      throw new Error(`AGiXT conversation history failed: ${response.statusText}`);
    }

    const data = await response.json();
    return (data.conversation || []).map((msg: { role: string; content: string }) => ({
      role: msg.role as 'system' | 'user' | 'assistant',
      content: msg.content,
    }));
  }

  private getHeaders(): HeadersInit {
    const headers: HeadersInit = {
      'Content-Type': 'application/json',
    };
    if (this.config.apiKey) {
      headers['Authorization'] = `Bearer ${this.config.apiKey}`;
    }
    return headers;
  }
}

/**
 * Unified AI Service - Combines LocalAI and AGiXT
 */
export class AIService {
  public localai: LocalAIService;
  public agixt: AGiXTService;

  constructor(localaiConfig?: Partial<LocalAIConfig>, agixtConfig?: Partial<AGiXTConfig>) {
    this.localai = new LocalAIService(localaiConfig);
    this.agixt = new AGiXTService(agixtConfig);
  }

  /**
   * Check all AI services health
   */
  async healthCheck(): Promise<{ localai: boolean; agixt: boolean }> {
    const [localaiHealth, agixtHealth] = await Promise.all([
      this.localai.healthCheck(),
      this.agixt.healthCheck(),
    ]);
    return { localai: localaiHealth, agixt: agixtHealth };
  }

  /**
   * Smart completion - uses LocalAI for simple completions, AGiXT for complex tasks
   */
  async smartCompletion(
    messages: AIMessage[],
    options?: AICompletionOptions & { useAgent?: boolean; agentName?: string }
  ): Promise<string> {
    // If agent mode is requested or message seems complex, use AGiXT
    if (options?.useAgent) {
      const lastUserMessage = messages.filter(m => m.role === 'user').pop();
      if (lastUserMessage) {
        const response = await this.agixt.chat(lastUserMessage.content, undefined, options.agentName);
        return response.response;
      }
    }

    // Otherwise use LocalAI
    const response = await this.localai.chatCompletion(messages, options);
    return response.content;
  }
}

// Singleton instances
export const localAI = new LocalAIService();
export const agixt = new AGiXTService();
export const aiService = new AIService();

// React hook for AI services
import { useState, useEffect, useCallback } from 'react';

export interface AIServiceStatus {
  localai: boolean;
  agixt: boolean;
  checking: boolean;
}

export function useAIServices() {
  const [status, setStatus] = useState<AIServiceStatus>({
    localai: false,
    agixt: false,
    checking: true,
  });

  const checkHealth = useCallback(async () => {
    setStatus(prev => ({ ...prev, checking: true }));
    const health = await aiService.healthCheck();
    setStatus({ ...health, checking: false });
    return health;
  }, []);

  useEffect(() => {
    checkHealth();
    // Recheck every 30 seconds
    const interval = setInterval(checkHealth, 30000);
    return () => clearInterval(interval);
  }, [checkHealth]);

  return {
    status,
    checkHealth,
    localai: aiService.localai,
    agixt: aiService.agixt,
    smartCompletion: aiService.smartCompletion.bind(aiService),
  };
}
