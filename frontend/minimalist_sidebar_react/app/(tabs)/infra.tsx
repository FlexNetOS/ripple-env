import { ScrollView, Text, View, Pressable } from "react-native";
import { useState } from "react";
import { ScreenContainer } from "@/components/screen-container";
import { useColors } from "@/hooks/use-colors";
import { IconSymbol } from "@/components/ui/icon-symbol";

// Mock infrastructure data
const nodes = [
  {
    id: "1",
    name: "ripple-master",
    type: "Master",
    status: "healthy" as const,
    cpu: 45,
    memory: 62,
    storage: 38,
    services: ["K8s Control Plane", "etcd", "API Server"],
  },
  {
    id: "2",
    name: "ripple-worker-01",
    type: "Worker",
    status: "healthy" as const,
    cpu: 78,
    memory: 85,
    storage: 45,
    services: ["LocalAI", "Postgres", "Redis"],
  },
  {
    id: "3",
    name: "ripple-worker-02",
    type: "Worker",
    status: "healthy" as const,
    cpu: 56,
    memory: 71,
    storage: 52,
    services: ["AGiXT", "Temporal", "NATS"],
  },
  {
    id: "4",
    name: "ripple-worker-03",
    type: "Worker",
    status: "warning" as const,
    cpu: 92,
    memory: 88,
    storage: 67,
    services: ["MinIO", "Qdrant", "n8n"],
  },
  {
    id: "5",
    name: "ripple-inference",
    type: "GPU",
    status: "healthy" as const,
    cpu: 34,
    memory: 56,
    storage: 28,
    services: ["CUDA Runtime", "Model Server"],
  },
];

const services = [
  { name: "Kubernetes", status: "running" as const, version: "1.29.0" },
  { name: "LocalAI", status: "running" as const, version: "2.5.1" },
  { name: "PostgreSQL", status: "running" as const, version: "16.1" },
  { name: "Redis", status: "running" as const, version: "7.2.3" },
  { name: "MinIO", status: "running" as const, version: "2024.01" },
  { name: "Temporal", status: "running" as const, version: "1.22.0" },
  { name: "NATS", status: "running" as const, version: "2.10.0" },
  { name: "Holochain", status: "starting" as const, version: "0.3.0" },
];

type NodeStatus = "healthy" | "warning" | "error";
type ServiceStatus = "running" | "starting" | "stopped" | "error";

function ResourceBar({ label, value, color }: { label: string; value: number; color: string }) {
  const colors = useColors();
  
  return (
    <View className="mb-2">
      <View className="flex-row justify-between mb-1">
        <Text className="text-xs text-muted">{label}</Text>
        <Text className="text-xs font-medium text-foreground">{value}%</Text>
      </View>
      <View className="h-1.5 bg-border rounded-full overflow-hidden">
        <View 
          className="h-full rounded-full"
          style={{ 
            width: `${value}%`,
            backgroundColor: value > 85 ? colors.error : value > 70 ? colors.warning : color,
          }}
        />
      </View>
    </View>
  );
}

function NodeCard({ node }: { node: typeof nodes[0] }) {
  const colors = useColors();
  
  const statusConfig: Record<NodeStatus, { color: string; label: string }> = {
    healthy: { color: colors.success, label: "Healthy" },
    warning: { color: colors.warning, label: "Warning" },
    error: { color: colors.error, label: "Error" },
  };

  const config = statusConfig[node.status];

  return (
    <View 
      className="p-4 rounded-xl mb-3"
      style={{ backgroundColor: colors.surface }}
    >
      <View className="flex-row items-center justify-between mb-3">
        <View className="flex-row items-center gap-3">
          <View 
            className="w-10 h-10 rounded-full items-center justify-center"
            style={{ backgroundColor: `${config.color}20` }}
          >
            <IconSymbol name="server.rack" size={20} color={config.color} />
          </View>
          <View>
            <Text className="text-base font-semibold text-foreground">{node.name}</Text>
            <Text className="text-xs text-muted">{node.type} Node</Text>
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

      <ResourceBar label="CPU" value={node.cpu} color={colors.primary} />
      <ResourceBar label="Memory" value={node.memory} color={colors.secondary} />
      <ResourceBar label="Storage" value={node.storage} color={colors.accent} />

      <View className="flex-row flex-wrap gap-1 mt-3">
        {node.services.map((service, index) => (
          <View 
            key={index}
            className="px-2 py-1 rounded"
            style={{ backgroundColor: colors.background }}
          >
            <Text className="text-xs text-muted">{service}</Text>
          </View>
        ))}
      </View>
    </View>
  );
}

function ServiceRow({ service }: { service: typeof services[0] }) {
  const colors = useColors();
  
  const statusConfig: Record<ServiceStatus, { color: string; label: string }> = {
    running: { color: colors.success, label: "Running" },
    starting: { color: colors.warning, label: "Starting" },
    stopped: { color: colors.muted, label: "Stopped" },
    error: { color: colors.error, label: "Error" },
  };

  const config = statusConfig[service.status];

  return (
    <View 
      className="flex-row items-center justify-between py-3"
      style={{ borderBottomWidth: 1, borderBottomColor: colors.border }}
    >
      <View className="flex-row items-center gap-3">
        <View 
          className="w-2 h-2 rounded-full"
          style={{ backgroundColor: config.color }}
        />
        <Text className="text-sm text-foreground">{service.name}</Text>
      </View>
      <View className="flex-row items-center gap-3">
        <Text className="text-xs text-muted">v{service.version}</Text>
        <Text className="text-xs" style={{ color: config.color }}>{config.label}</Text>
      </View>
    </View>
  );
}

export default function InfraScreen() {
  const colors = useColors();
  const [activeTab, setActiveTab] = useState<"nodes" | "services">("nodes");

  const healthyNodes = nodes.filter(n => n.status === "healthy").length;
  const runningServices = services.filter(s => s.status === "running").length;

  return (
    <ScreenContainer>
      <ScrollView 
        className="flex-1"
        contentContainerStyle={{ paddingBottom: 24 }}
        showsVerticalScrollIndicator={false}
      >
        {/* Header */}
        <View className="px-5 pt-4 pb-6">
          <Text className="text-2xl font-bold text-foreground">Infrastructure</Text>
          <Text className="text-sm text-muted mt-1">Distributed System Status</Text>
        </View>

        {/* Stats */}
        <View className="px-5 mb-6">
          <View className="flex-row gap-3">
            <View 
              className="flex-1 p-4 rounded-xl"
              style={{ backgroundColor: colors.surface }}
            >
              <Text className="text-2xl font-bold" style={{ color: colors.success }}>
                {healthyNodes}/{nodes.length}
              </Text>
              <Text className="text-xs text-muted">Nodes Healthy</Text>
            </View>
            <View 
              className="flex-1 p-4 rounded-xl"
              style={{ backgroundColor: colors.surface }}
            >
              <Text className="text-2xl font-bold" style={{ color: colors.primary }}>
                {runningServices}/{services.length}
              </Text>
              <Text className="text-xs text-muted">Services Running</Text>
            </View>
          </View>
        </View>

        {/* Tab Selector */}
        <View className="px-5 mb-4">
          <View 
            className="flex-row p-1 rounded-xl"
            style={{ backgroundColor: colors.surface }}
          >
            <Pressable
              onPress={() => setActiveTab("nodes")}
              style={({ pressed }) => [
                {
                  flex: 1,
                  paddingVertical: 10,
                  borderRadius: 10,
                  backgroundColor: activeTab === "nodes" ? colors.background : "transparent",
                  opacity: pressed ? 0.8 : 1,
                },
              ]}
            >
              <Text 
                className="text-sm font-medium text-center"
                style={{ color: activeTab === "nodes" ? colors.foreground : colors.muted }}
              >
                Nodes ({nodes.length})
              </Text>
            </Pressable>
            <Pressable
              onPress={() => setActiveTab("services")}
              style={({ pressed }) => [
                {
                  flex: 1,
                  paddingVertical: 10,
                  borderRadius: 10,
                  backgroundColor: activeTab === "services" ? colors.background : "transparent",
                  opacity: pressed ? 0.8 : 1,
                },
              ]}
            >
              <Text 
                className="text-sm font-medium text-center"
                style={{ color: activeTab === "services" ? colors.foreground : colors.muted }}
              >
                Services ({services.length})
              </Text>
            </Pressable>
          </View>
        </View>

        {/* Content */}
        <View className="px-5">
          {activeTab === "nodes" ? (
            <>
              {nodes.map((node) => (
                <NodeCard key={node.id} node={node} />
              ))}
            </>
          ) : (
            <View 
              className="rounded-xl overflow-hidden"
              style={{ backgroundColor: colors.surface }}
            >
              <View className="px-4">
                {services.map((service, index) => (
                  <ServiceRow key={index} service={service} />
                ))}
              </View>
            </View>
          )}
        </View>
      </ScrollView>
    </ScreenContainer>
  );
}
