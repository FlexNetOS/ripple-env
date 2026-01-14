import { ScrollView, Text, View, Pressable } from "react-native";
import { useState, useEffect } from "react";
import { ScreenContainer } from "@/components/screen-container";
import { useColors } from "@/hooks/use-colors";
import { StatusCard } from "@/components/status-card";
import { QuickActionCard } from "@/components/quick-action-card";
import { ActivityItem } from "@/components/activity-item";

// Mock data for system status
const systemStatus = {
  health: "healthy" as const,
  uptime: "99.9%",
  activeAgents: 5,
  runningTasks: 12,
  currentPhase: "P3",
  buildProgress: 45,
};

const recentActivity = [
  { id: "1", type: "success" as const, message: "P2 Model Cluster completed", time: "2m ago" },
  { id: "2", type: "info" as const, message: "Agent AGiXT-01 started task", time: "5m ago" },
  { id: "3", type: "warning" as const, message: "High memory usage on Node-03", time: "12m ago" },
  { id: "4", type: "success" as const, message: "Vendor hashes verified (69/69)", time: "18m ago" },
  { id: "5", type: "info" as const, message: "LocalAI inference plane ready", time: "25m ago" },
];

export default function HomeScreen() {
  const colors = useColors();
  const [currentTime, setCurrentTime] = useState(new Date());

  useEffect(() => {
    const timer = setInterval(() => setCurrentTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  return (
    <ScreenContainer>
      <ScrollView 
        className="flex-1"
        contentContainerStyle={{ paddingBottom: 24 }}
        showsVerticalScrollIndicator={false}
      >
        {/* Header with gradient */}
        <View className="px-5 pt-4 pb-6">
          <View className="flex-row items-center justify-between mb-2">
            <View>
              <Text className="text-3xl font-bold text-foreground">Ripple</Text>
              <Text className="text-sm text-muted">Agentic DevOps Platform</Text>
            </View>
            <View className="items-end">
              <Text className="text-xs text-muted">
                {currentTime.toLocaleDateString()}
              </Text>
              <Text className="text-lg font-semibold text-foreground">
                {currentTime.toLocaleTimeString()}
              </Text>
            </View>
          </View>
        </View>

        {/* System Health Banner */}
        <View className="px-5 mb-6">
          <View 
            className="rounded-2xl overflow-hidden"
            style={{ backgroundColor: colors.surface }}
          >
            <View 
              className="h-1"
              style={{ 
                backgroundColor: systemStatus.health === "healthy" ? colors.success : colors.warning 
              }}
            />
            <View className="p-4">
              <View className="flex-row items-center justify-between">
                <View className="flex-row items-center gap-3">
                  <View 
                    className="w-3 h-3 rounded-full"
                    style={{ 
                      backgroundColor: systemStatus.health === "healthy" ? colors.success : colors.warning 
                    }}
                  />
                  <Text className="text-lg font-semibold text-foreground">
                    System {systemStatus.health === "healthy" ? "Healthy" : "Warning"}
                  </Text>
                </View>
                <Text className="text-sm text-muted">
                  Uptime: {systemStatus.uptime}
                </Text>
              </View>
              
              {/* Build Progress */}
              <View className="mt-4">
                <View className="flex-row justify-between mb-2">
                  <Text className="text-sm text-muted">Build Progress</Text>
                  <Text className="text-sm font-medium text-foreground">
                    Phase {systemStatus.currentPhase} • {systemStatus.buildProgress}%
                  </Text>
                </View>
                <View className="h-2 bg-border rounded-full overflow-hidden">
                  <View 
                    className="h-full rounded-full"
                    style={{ 
                      width: `${systemStatus.buildProgress}%`,
                      backgroundColor: colors.primary 
                    }}
                  />
                </View>
              </View>
            </View>
          </View>
        </View>

        {/* Status Cards Grid */}
        <View className="px-5 mb-6">
          <Text className="text-lg font-semibold text-foreground mb-3">Overview</Text>
          <View className="flex-row flex-wrap gap-3">
            <StatusCard
              title="Active Agents"
              value={systemStatus.activeAgents.toString()}
              subtitle="5 online"
              status="success"
              icon="person.2.fill"
            />
            <StatusCard
              title="Running Tasks"
              value={systemStatus.runningTasks.toString()}
              subtitle="3 queued"
              status="info"
              icon="bolt.fill"
            />
            <StatusCard
              title="Build Phase"
              value={systemStatus.currentPhase}
              subtitle="Inference & State"
              status="info"
              icon="hammer.fill"
            />
            <StatusCard
              title="Nodes"
              value="8"
              subtitle="All healthy"
              status="success"
              icon="server.rack"
            />
          </View>
        </View>

        {/* Quick Actions */}
        <View className="px-5 mb-6">
          <Text className="text-lg font-semibold text-foreground mb-3">Quick Actions</Text>
          <ScrollView 
            horizontal 
            showsHorizontalScrollIndicator={false}
            contentContainerStyle={{ gap: 12 }}
          >
            <QuickActionCard
              title="Start Build"
              icon="play.fill"
              color={colors.success}
            />
            <QuickActionCard
              title="View Logs"
              icon="doc.text.fill"
              color={colors.primary}
            />
            <QuickActionCard
              title="Deploy"
              icon="paperplane.fill"
              color={colors.secondary}
            />
            <QuickActionCard
              title="Monitor"
              icon="chart.bar.fill"
              color={colors.accent}
            />
          </ScrollView>
        </View>

        {/* Recent Activity */}
        <View className="px-5">
          <View className="flex-row items-center justify-between mb-3">
            <Text className="text-lg font-semibold text-foreground">Recent Activity</Text>
            <Pressable>
              <Text className="text-sm" style={{ color: colors.primary }}>View All</Text>
            </Pressable>
          </View>
          <View 
            className="rounded-2xl overflow-hidden"
            style={{ backgroundColor: colors.surface }}
          >
            {recentActivity.map((activity, index) => (
              <ActivityItem
                key={activity.id}
                type={activity.type}
                message={activity.message}
                time={activity.time}
                isLast={index === recentActivity.length - 1}
              />
            ))}
          </View>
        </View>
      </ScrollView>
    </ScreenContainer>
  );
}
