import { ScrollView, Text, View, Pressable } from "react-native";
import { useState } from "react";
import { ScreenContainer } from "@/components/screen-container";
import { useColors } from "@/hooks/use-colors";
import { IconSymbol } from "@/components/ui/icon-symbol";

// Build phases from BuildKit specification
const buildPhases = [
  { 
    id: "P0", 
    name: "Environment Foundation", 
    status: "completed" as const,
    description: "Initialize workspace, verify tools, validate inputs",
    tasks: 8,
    completedTasks: 8,
  },
  { 
    id: "P1", 
    name: "Vendor Integration", 
    status: "completed" as const,
    description: "Extract repos, set up AGiXT, compile binary",
    tasks: 12,
    completedTasks: 12,
  },
  { 
    id: "P2", 
    name: "Model Cluster", 
    status: "completed" as const,
    description: "Download models, bootstrap K8s, install GitOps",
    tasks: 6,
    completedTasks: 6,
  },
  { 
    id: "P3", 
    name: "Inference & State", 
    status: "running" as const,
    description: "Set up LocalAI, deploy state plane",
    tasks: 8,
    completedTasks: 4,
  },
  { 
    id: "P4", 
    name: "Database & Inference", 
    status: "pending" as const,
    description: "Configure databases, deploy MOE inference",
    tasks: 10,
    completedTasks: 0,
  },
  { 
    id: "P5", 
    name: "Orchestration", 
    status: "pending" as const,
    description: "Deploy Temporal, n8n, NATS",
    tasks: 6,
    completedTasks: 0,
  },
  { 
    id: "P6", 
    name: "Testing & Promotion", 
    status: "pending" as const,
    description: "Run tests, evals, QA gates",
    tasks: 8,
    completedTasks: 0,
  },
  { 
    id: "P7", 
    name: "Build & Package", 
    status: "pending" as const,
    description: "Build, sign, package, distribute",
    tasks: 5,
    completedTasks: 0,
  },
];

type PhaseStatus = "completed" | "running" | "pending" | "failed";

interface PhaseCardProps {
  phase: typeof buildPhases[0];
  isExpanded: boolean;
  onToggle: () => void;
}

function PhaseCard({ phase, isExpanded, onToggle }: PhaseCardProps) {
  const colors = useColors();
  
  const statusConfig: Record<PhaseStatus, { color: string; icon: "checkmark.circle.fill" | "arrow.clockwise" | "clock.fill" | "xmark.circle.fill"; label: string }> = {
    completed: { color: colors.success, icon: "checkmark.circle.fill", label: "Completed" },
    running: { color: colors.primary, icon: "arrow.clockwise", label: "Running" },
    pending: { color: colors.muted, icon: "clock.fill", label: "Pending" },
    failed: { color: colors.error, icon: "xmark.circle.fill", label: "Failed" },
  };

  const config = statusConfig[phase.status];
  const progress = phase.tasks > 0 ? (phase.completedTasks / phase.tasks) * 100 : 0;

  return (
    <Pressable
      onPress={onToggle}
      style={({ pressed }) => [
        {
          backgroundColor: colors.surface,
          borderRadius: 16,
          marginBottom: 12,
          overflow: "hidden",
          opacity: pressed ? 0.9 : 1,
        },
      ]}
    >
      {/* Phase Header */}
      <View className="p-4">
        <View className="flex-row items-center justify-between mb-2">
          <View className="flex-row items-center gap-3">
            <View 
              className="w-10 h-10 rounded-full items-center justify-center"
              style={{ backgroundColor: `${config.color}20` }}
            >
              <Text className="font-bold" style={{ color: config.color }}>{phase.id}</Text>
            </View>
            <View className="flex-1">
              <Text className="text-base font-semibold text-foreground">{phase.name}</Text>
              <Text className="text-xs" style={{ color: config.color }}>{config.label}</Text>
            </View>
          </View>
          <IconSymbol 
            name={isExpanded ? "chevron.right" : "chevron.right"} 
            size={20} 
            color={colors.muted}
            style={{ transform: [{ rotate: isExpanded ? '90deg' : '0deg' }] }}
          />
        </View>
        
        {/* Progress Bar */}
        <View className="h-2 bg-border rounded-full overflow-hidden mt-2">
          <View 
            className="h-full rounded-full"
            style={{ 
              width: `${progress}%`,
              backgroundColor: config.color,
            }}
          />
        </View>
        <Text className="text-xs text-muted mt-1">
          {phase.completedTasks}/{phase.tasks} tasks
        </Text>
      </View>

      {/* Expanded Content */}
      {isExpanded && (
        <View 
          className="px-4 pb-4 pt-2"
          style={{ borderTopWidth: 1, borderTopColor: colors.border }}
        >
          <Text className="text-sm text-muted mb-3">{phase.description}</Text>
          
          {phase.status === "running" && (
            <View className="flex-row gap-2">
              <Pressable
                style={({ pressed }) => [
                  {
                    backgroundColor: colors.primary,
                    paddingHorizontal: 16,
                    paddingVertical: 8,
                    borderRadius: 8,
                    opacity: pressed ? 0.8 : 1,
                  },
                ]}
              >
                <Text className="text-sm font-medium text-background">View Logs</Text>
              </Pressable>
              <Pressable
                style={({ pressed }) => [
                  {
                    backgroundColor: colors.error,
                    paddingHorizontal: 16,
                    paddingVertical: 8,
                    borderRadius: 8,
                    opacity: pressed ? 0.8 : 1,
                  },
                ]}
              >
                <Text className="text-sm font-medium text-background">Stop</Text>
              </Pressable>
            </View>
          )}
          
          {phase.status === "pending" && (
            <Pressable
              style={({ pressed }) => [
                {
                  backgroundColor: `${colors.primary}20`,
                  paddingHorizontal: 16,
                  paddingVertical: 8,
                  borderRadius: 8,
                  alignSelf: "flex-start",
                  opacity: pressed ? 0.8 : 1,
                },
              ]}
            >
              <Text className="text-sm font-medium" style={{ color: colors.primary }}>
                Start Phase
              </Text>
            </Pressable>
          )}
        </View>
      )}
    </Pressable>
  );
}

export default function BuildScreen() {
  const colors = useColors();
  const [expandedPhase, setExpandedPhase] = useState<string | null>("P3");

  const completedPhases = buildPhases.filter(p => p.status === "completed").length;
  const totalProgress = Math.round((completedPhases / buildPhases.length) * 100);

  return (
    <ScreenContainer>
      <ScrollView 
        className="flex-1"
        contentContainerStyle={{ paddingBottom: 24 }}
        showsVerticalScrollIndicator={false}
      >
        {/* Header */}
        <View className="px-5 pt-4 pb-6">
          <Text className="text-2xl font-bold text-foreground">Build Pipeline</Text>
          <Text className="text-sm text-muted mt-1">P0 → P7 Phase Execution</Text>
        </View>

        {/* Overall Progress */}
        <View className="px-5 mb-6">
          <View 
            className="p-4 rounded-2xl"
            style={{ backgroundColor: colors.surface }}
          >
            <View className="flex-row items-center justify-between mb-3">
              <Text className="text-base font-semibold text-foreground">Overall Progress</Text>
              <Text className="text-2xl font-bold" style={{ color: colors.primary }}>
                {totalProgress}%
              </Text>
            </View>
            <View className="h-3 bg-border rounded-full overflow-hidden">
              <View 
                className="h-full rounded-full"
                style={{ 
                  width: `${totalProgress}%`,
                  backgroundColor: colors.primary,
                }}
              />
            </View>
            <Text className="text-xs text-muted mt-2">
              {completedPhases} of {buildPhases.length} phases completed
            </Text>
          </View>
        </View>

        {/* Phase List */}
        <View className="px-5">
          <Text className="text-lg font-semibold text-foreground mb-3">Phases</Text>
          {buildPhases.map((phase) => (
            <PhaseCard
              key={phase.id}
              phase={phase}
              isExpanded={expandedPhase === phase.id}
              onToggle={() => setExpandedPhase(expandedPhase === phase.id ? null : phase.id)}
            />
          ))}
        </View>
      </ScrollView>
    </ScreenContainer>
  );
}
