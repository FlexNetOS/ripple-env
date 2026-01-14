import { ScrollView, Text, View, Pressable } from "react-native";
import { useState } from "react";
import { ScreenContainer } from "@/components/screen-container";
import { useColors } from "@/hooks/use-colors";
import { IconSymbol } from "@/components/ui/icon-symbol";

// Mock agent data based on BuildKit MOE policy
const agents = [
  {
    id: "1",
    name: "AGiXT-01",
    type: "Orchestrator",
    status: "online" as const,
    currentTask: "P3: Deploy state plane",
    tasksCompleted: 24,
    model: "gemma-3n-E2B-it",
  },
  {
    id: "2",
    name: "LocalAI-Primary",
    type: "Inference",
    status: "online" as const,
    currentTask: "Model loading",
    tasksCompleted: 156,
    model: "Phi-4-mini-reasoning",
  },
  {
    id: "3",
    name: "DeepSeek-R1",
    type: "Reasoning",
    status: "online" as const,
    currentTask: "Code review",
    tasksCompleted: 89,
    model: "DeepSeek-R1-Distill-Qwen-1.5B",
  },
  {
    id: "4",
    name: "Qwen-Builder",
    type: "Builder",
    status: "busy" as const,
    currentTask: "Cross-compile ARM64",
    tasksCompleted: 45,
    model: "Qwen3-4B",
  },
  {
    id: "5",
    name: "Gemma-Validator",
    type: "Validator",
    status: "online" as const,
    currentTask: "Hash verification",
    tasksCompleted: 312,
    model: "gemma-3-1b-it",
  },
  {
    id: "6",
    name: "Claude-Code",
    type: "Cloud",
    status: "online" as const,
    currentTask: "Idle",
    tasksCompleted: 67,
    model: "Claude Code CLI",
  },
  {
    id: "7",
    name: "Codex-Agent",
    type: "Cloud",
    status: "offline" as const,
    currentTask: "None",
    tasksCompleted: 23,
    model: "Codex CLI",
  },
];

type AgentStatus = "online" | "offline" | "busy";

interface AgentCardProps {
  agent: typeof agents[0];
  isSelected: boolean;
  onSelect: () => void;
}

function AgentCard({ agent, isSelected, onSelect }: AgentCardProps) {
  const colors = useColors();
  
  const statusConfig: Record<AgentStatus, { color: string; label: string }> = {
    online: { color: colors.success, label: "Online" },
    offline: { color: colors.muted, label: "Offline" },
    busy: { color: colors.warning, label: "Busy" },
  };

  const config = statusConfig[agent.status];

  return (
    <Pressable
      onPress={onSelect}
      style={({ pressed }) => [
        {
          backgroundColor: colors.surface,
          borderRadius: 16,
          marginBottom: 12,
          borderWidth: isSelected ? 2 : 0,
          borderColor: isSelected ? colors.primary : "transparent",
          opacity: pressed ? 0.9 : 1,
        },
      ]}
    >
      <View className="p-4">
        <View className="flex-row items-center justify-between mb-3">
          <View className="flex-row items-center gap-3">
            <View 
              className="w-10 h-10 rounded-full items-center justify-center"
              style={{ backgroundColor: `${colors.primary}20` }}
            >
              <IconSymbol name="person.2.fill" size={20} color={colors.primary} />
            </View>
            <View>
              <Text className="text-base font-semibold text-foreground">{agent.name}</Text>
              <Text className="text-xs text-muted">{agent.type}</Text>
            </View>
          </View>
          <View className="flex-row items-center gap-2">
            <View 
              className="w-2 h-2 rounded-full"
              style={{ backgroundColor: config.color }}
            />
            <Text className="text-xs" style={{ color: config.color }}>{config.label}</Text>
          </View>
        </View>

        <View 
          className="p-3 rounded-lg"
          style={{ backgroundColor: colors.background }}
        >
          <Text className="text-xs text-muted mb-1">Current Task</Text>
          <Text className="text-sm text-foreground">{agent.currentTask}</Text>
        </View>

        <View className="flex-row justify-between mt-3">
          <View>
            <Text className="text-xs text-muted">Model</Text>
            <Text className="text-xs text-foreground">{agent.model}</Text>
          </View>
          <View className="items-end">
            <Text className="text-xs text-muted">Tasks Completed</Text>
            <Text className="text-sm font-semibold" style={{ color: colors.primary }}>
              {agent.tasksCompleted}
            </Text>
          </View>
        </View>
      </View>
    </Pressable>
  );
}

export default function AgentsScreen() {
  const colors = useColors();
  const [selectedAgent, setSelectedAgent] = useState<string | null>(null);

  const onlineCount = agents.filter(a => a.status === "online").length;
  const busyCount = agents.filter(a => a.status === "busy").length;

  return (
    <ScreenContainer>
      <ScrollView 
        className="flex-1"
        contentContainerStyle={{ paddingBottom: 24 }}
        showsVerticalScrollIndicator={false}
      >
        {/* Header */}
        <View className="px-5 pt-4 pb-6">
          <Text className="text-2xl font-bold text-foreground">Agents</Text>
          <Text className="text-sm text-muted mt-1">MOE Inference Cluster</Text>
        </View>

        {/* Stats */}
        <View className="px-5 mb-6">
          <View className="flex-row gap-3">
            <View 
              className="flex-1 p-4 rounded-xl"
              style={{ backgroundColor: colors.surface }}
            >
              <Text className="text-2xl font-bold" style={{ color: colors.success }}>
                {onlineCount}
              </Text>
              <Text className="text-xs text-muted">Online</Text>
            </View>
            <View 
              className="flex-1 p-4 rounded-xl"
              style={{ backgroundColor: colors.surface }}
            >
              <Text className="text-2xl font-bold" style={{ color: colors.warning }}>
                {busyCount}
              </Text>
              <Text className="text-xs text-muted">Busy</Text>
            </View>
            <View 
              className="flex-1 p-4 rounded-xl"
              style={{ backgroundColor: colors.surface }}
            >
              <Text className="text-2xl font-bold" style={{ color: colors.primary }}>
                {agents.length}
              </Text>
              <Text className="text-xs text-muted">Total</Text>
            </View>
          </View>
        </View>

        {/* MOE Policy Info */}
        <View className="px-5 mb-4">
          <View 
            className="p-3 rounded-xl flex-row items-center gap-3"
            style={{ backgroundColor: `${colors.primary}15` }}
          >
            <IconSymbol name="info.circle.fill" size={20} color={colors.primary} />
            <Text className="text-xs flex-1" style={{ color: colors.primary }}>
              MOE Policy: 5+ local models + 2+ cloud providers with 60% consensus threshold
            </Text>
          </View>
        </View>

        {/* Agent List */}
        <View className="px-5">
          <View className="flex-row items-center justify-between mb-3">
            <Text className="text-lg font-semibold text-foreground">All Agents</Text>
            <Pressable
              style={({ pressed }) => [
                {
                  backgroundColor: colors.primary,
                  paddingHorizontal: 12,
                  paddingVertical: 6,
                  borderRadius: 8,
                  opacity: pressed ? 0.8 : 1,
                },
              ]}
            >
              <Text className="text-xs font-medium text-background">+ Add Agent</Text>
            </Pressable>
          </View>
          
          {agents.map((agent) => (
            <AgentCard
              key={agent.id}
              agent={agent}
              isSelected={selectedAgent === agent.id}
              onSelect={() => setSelectedAgent(selectedAgent === agent.id ? null : agent.id)}
            />
          ))}
        </View>
      </ScrollView>
    </ScreenContainer>
  );
}
