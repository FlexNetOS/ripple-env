// Agent selector component for choosing coding agents
import React from 'react';
import { View, Text, Pressable, ScrollView } from 'react-native';
import type { BaseCodingAgent } from '@/shared/vibe-kanban/types';

interface AgentOption {
  id: BaseCodingAgent;
  name: string;
  description: string;
  icon: string;
  available: boolean;
}

const AGENTS: AgentOption[] = [
  {
    id: 'CLAUDE_CODE',
    name: 'Claude Code',
    description: 'Anthropic\'s coding assistant',
    icon: '🤖',
    available: true,
  },
  {
    id: 'CODEX',
    name: 'Codex',
    description: 'OpenAI\'s code model',
    icon: '🔮',
    available: true,
  },
  {
    id: 'GEMINI_CLI',
    name: 'Gemini CLI',
    description: 'Google\'s Gemini model',
    icon: '✨',
    available: true,
  },
  {
    id: 'CUSTOM',
    name: 'Custom Agent',
    description: 'Use a custom configuration',
    icon: '⚙️',
    available: true,
  },
];

interface AgentSelectorProps {
  selectedAgent: BaseCodingAgent | null;
  onSelectAgent: (agent: BaseCodingAgent) => void;
  disabled?: boolean;
}

export function AgentSelector({ selectedAgent, onSelectAgent, disabled }: AgentSelectorProps) {
  return (
    <View style={{ gap: 8 }}>
      <Text style={{ color: '#9ca3af', fontSize: 12, textTransform: 'uppercase', letterSpacing: 0.5 }}>
        Select Agent
      </Text>
      <ScrollView horizontal showsHorizontalScrollIndicator={false}>
        <View style={{ flexDirection: 'row', gap: 8 }}>
          {AGENTS.map(agent => (
            <Pressable
              key={agent.id}
              onPress={() => !disabled && agent.available && onSelectAgent(agent.id)}
              disabled={disabled || !agent.available}
              style={({ pressed }) => ({
                backgroundColor: selectedAgent === agent.id ? 'rgba(59, 130, 246, 0.2)' : '#242424',
                borderWidth: 1,
                borderColor: selectedAgent === agent.id ? '#3b82f6' : 'rgba(255, 255, 255, 0.1)',
                borderRadius: 8,
                padding: 12,
                minWidth: 140,
                opacity: !agent.available || disabled ? 0.5 : pressed ? 0.8 : 1,
              })}
            >
              <Text style={{ fontSize: 24, marginBottom: 8 }}>{agent.icon}</Text>
              <Text style={{ color: '#ffffff', fontSize: 14, fontWeight: '500', marginBottom: 2 }}>
                {agent.name}
              </Text>
              <Text style={{ color: '#9ca3af', fontSize: 11 }} numberOfLines={1}>
                {agent.description}
              </Text>
              {!agent.available && (
                <View
                  style={{
                    position: 'absolute',
                    top: 8,
                    right: 8,
                    backgroundColor: 'rgba(239, 68, 68, 0.2)',
                    paddingHorizontal: 6,
                    paddingVertical: 2,
                    borderRadius: 4,
                  }}
                >
                  <Text style={{ color: '#ef4444', fontSize: 10 }}>Unavailable</Text>
                </View>
              )}
            </Pressable>
          ))}
        </View>
      </ScrollView>
    </View>
  );
}
