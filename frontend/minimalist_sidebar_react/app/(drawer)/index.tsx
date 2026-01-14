import React, { useState, useEffect } from 'react';
import { ScrollView, Text, View, Pressable, StyleSheet } from 'react-native';
import { ScreenContainer } from '@/components/screen-container';
import { useColors } from '@/hooks/use-colors';
import { DrawerActions, useNavigation } from '@react-navigation/native';
import MaterialIcons from '@expo/vector-icons/MaterialIcons';
import Animated, { useAnimatedStyle, withRepeat, withTiming, withSequence } from 'react-native-reanimated';
import { 
  realtimeData, 
  useSystemHealth, 
  useRealtimeBuild, 
  useRealtimeAgents,
  type Alert 
} from '@/services/realtime-data';

// Status Card Component
function StatusCard({ title, value, subtitle, status, icon }: {
  title: string;
  value: string | number;
  subtitle: string;
  status: 'success' | 'warning' | 'error' | 'info';
  icon: keyof typeof MaterialIcons.glyphMap;
}) {
  const colors = useColors();
  const statusColors = {
    success: '#00E676',
    warning: '#FFB300',
    error: '#FF5252',
    info: '#00D4FF',
  };

  return (
    <View style={[styles.statusCard, { backgroundColor: colors.surface, borderColor: colors.border }]}>
      <View style={styles.statusCardHeader}>
        <MaterialIcons name={icon} size={20} color={statusColors[status]} />
        <View style={[styles.statusDot, { backgroundColor: statusColors[status] }]} />
      </View>
      <Text style={[styles.statusValue, { color: colors.foreground }]}>{value}</Text>
      <Text style={[styles.statusTitle, { color: colors.foreground }]}>{title}</Text>
      <Text style={[styles.statusSubtitle, { color: statusColors[status] }]}>{subtitle}</Text>
    </View>
  );
}

// Quick Action Button
function QuickAction({ icon, label, onPress }: {
  icon: keyof typeof MaterialIcons.glyphMap;
  label: string;
  onPress?: () => void;
}) {
  const colors = useColors();

  return (
    <Pressable
      onPress={onPress}
      style={({ pressed }) => [
        styles.quickAction,
        { backgroundColor: colors.surface, borderColor: colors.border },
        pressed && { opacity: 0.7 },
      ]}
    >
      <View style={[styles.quickActionIcon, { backgroundColor: 'rgba(0, 212, 255, 0.1)' }]}>
        <MaterialIcons name={icon} size={24} color="#00D4FF" />
      </View>
      <Text style={[styles.quickActionLabel, { color: colors.foreground }]}>{label}</Text>
    </Pressable>
  );
}

// Activity Item
function ActivityItem({ icon, title, description, time, status }: {
  icon: keyof typeof MaterialIcons.glyphMap;
  title: string;
  description: string;
  time: string;
  status: 'success' | 'warning' | 'error' | 'info';
}) {
  const colors = useColors();
  const statusColors = {
    success: '#00E676',
    warning: '#FFB300',
    error: '#FF5252',
    info: '#00D4FF',
  };

  return (
    <View style={[styles.activityItem, { borderBottomColor: colors.border }]}>
      <View style={[styles.activityIcon, { backgroundColor: `${statusColors[status]}20` }]}>
        <MaterialIcons name={icon} size={16} color={statusColors[status]} />
      </View>
      <View style={styles.activityContent}>
        <Text style={[styles.activityTitle, { color: colors.foreground }]}>{title}</Text>
        <Text style={[styles.activityDescription, { color: colors.muted }]}>{description}</Text>
      </View>
      <Text style={[styles.activityTime, { color: colors.muted }]}>{time}</Text>
    </View>
  );
}

// Format time ago
function formatTimeAgo(date: Date): string {
  const seconds = Math.floor((Date.now() - date.getTime()) / 1000);
  if (seconds < 60) return `${seconds}s ago`;
  const minutes = Math.floor(seconds / 60);
  if (minutes < 60) return `${minutes}m ago`;
  const hours = Math.floor(minutes / 60);
  if (hours < 24) return `${hours}h ago`;
  return `${Math.floor(hours / 24)}d ago`;
}

export default function DashboardScreen() {
  const colors = useColors();
  const navigation = useNavigation();
  const [currentTime, setCurrentTime] = useState(new Date());
  
  // Real-time data hooks
  const health = useSystemHealth();
  const build = useRealtimeBuild();
  const agents = useRealtimeAgents();

  // Start real-time updates
  useEffect(() => {
    const stopUpdates = realtimeData.startAutoUpdate(3000);
    return stopUpdates;
  }, []);

  useEffect(() => {
    const timer = setInterval(() => setCurrentTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  const pulseStyle = useAnimatedStyle(() => ({
    opacity: withRepeat(
      withSequence(
        withTiming(1, { duration: 1000 }),
        withTiming(0.5, { duration: 1000 })
      ),
      -1,
      true
    ),
  }));

  // Determine status banner color based on health
  const healthColors = {
    healthy: { bg: 'rgba(0, 230, 118, 0.1)', border: '#00E676', text: 'System Healthy' },
    degraded: { bg: 'rgba(255, 179, 0, 0.1)', border: '#FFB300', text: 'System Degraded' },
    critical: { bg: 'rgba(255, 82, 82, 0.1)', border: '#FF5252', text: 'System Critical' },
  };
  const healthStatus = healthColors[health.overall];

  // Map alerts to activity items
  const recentAlerts = health.alerts.slice(0, 4).map((alert: Alert) => ({
    icon: alert.severity === 'error' || alert.severity === 'critical' ? 'error' : 
          alert.severity === 'warning' ? 'warning' : 'info',
    title: alert.title,
    description: alert.message,
    time: formatTimeAgo(alert.timestamp),
    status: alert.severity === 'error' || alert.severity === 'critical' ? 'error' :
            alert.severity === 'warning' ? 'warning' : 'info',
  }));

  return (
    <ScreenContainer containerClassName="bg-background">
      <ScrollView style={styles.container} showsVerticalScrollIndicator={false}>
        {/* Header */}
        <View style={styles.header}>
          <Pressable
            onPress={() => navigation.dispatch(DrawerActions.toggleDrawer())}
            style={styles.menuButton}
          >
            <MaterialIcons name="menu" size={24} color={colors.foreground} />
          </Pressable>
          <View style={styles.headerContent}>
            <Text style={[styles.title, { color: colors.foreground }]}>Ripple</Text>
            <Text style={[styles.subtitle, { color: colors.muted }]}>Agentic DevOps Platform</Text>
          </View>
          <View style={styles.headerRight}>
            <Text style={[styles.date, { color: colors.muted }]}>
              {currentTime.toLocaleDateString()}
            </Text>
            <Text style={[styles.time, { color: colors.foreground }]}>
              {currentTime.toLocaleTimeString()}
            </Text>
          </View>
        </View>

        {/* System Status Banner */}
        <View style={[styles.statusBanner, { backgroundColor: healthStatus.bg, borderColor: healthStatus.border }]}>
          <View style={styles.statusBannerLeft}>
            <Animated.View style={[styles.statusIndicator, { backgroundColor: healthStatus.border }, pulseStyle]} />
            <Text style={[styles.statusBannerTitle, { color: colors.foreground }]}>{healthStatus.text}</Text>
          </View>
          <Text style={[styles.statusBannerValue, { color: colors.muted }]}>
            {health.nodesOnline}/{health.totalNodes} nodes online
          </Text>
        </View>

        {/* Build Progress */}
        <View style={[styles.buildProgress, { backgroundColor: colors.surface, borderColor: colors.border }]}>
          <View style={styles.buildProgressHeader}>
            <Text style={[styles.buildProgressLabel, { color: colors.muted }]}>Build Progress</Text>
            <Text style={[styles.buildProgressPhase, { color: colors.foreground }]}>
              Phase {build.phases[build.currentPhase]?.id || 'P0'} • {build.overallProgress}%
            </Text>
          </View>
          <View style={[styles.progressBar, { backgroundColor: colors.border }]}>
            <View style={[styles.progressFill, { width: `${build.overallProgress}%` }]} />
          </View>
          <Text style={[styles.buildPhaseName, { color: colors.muted }]}>
            {build.phases[build.currentPhase]?.name || 'Initializing...'}
          </Text>
        </View>

        {/* Overview Cards */}
        <Text style={[styles.sectionTitle, { color: colors.foreground }]}>Overview</Text>
        <View style={styles.cardsGrid}>
          <StatusCard
            icon="groups"
            title="Active Agents"
            value={health.activeAgents}
            subtitle={`${health.totalAgents} total`}
            status={health.activeAgents === health.totalAgents ? 'success' : 'warning'}
          />
          <StatusCard
            icon="bolt"
            title="Running Tasks"
            value={health.runningTasks}
            subtitle={`${health.queuedTasks} queued`}
            status={health.queuedTasks > 20 ? 'warning' : 'info'}
          />
          <StatusCard
            icon="build"
            title="Build Phase"
            value={build.phases[build.currentPhase]?.id || 'P0'}
            subtitle={health.currentPhase}
            status={build.status === 'failed' ? 'error' : 'info'}
          />
          <StatusCard
            icon="dns"
            title="Nodes"
            value={health.nodesOnline}
            subtitle={health.overall === 'healthy' ? 'All healthy' : 'Issues detected'}
            status={health.overall === 'healthy' ? 'success' : health.overall === 'degraded' ? 'warning' : 'error'}
          />
        </View>

        {/* Quick Actions */}
        <Text style={[styles.sectionTitle, { color: colors.foreground }]}>Quick Actions</Text>
        <View style={styles.quickActionsRow}>
          <QuickAction icon="play-arrow" label="Start Build" />
          <QuickAction icon="description" label="View Logs" />
          <QuickAction icon="send" label="Deploy" />
        </View>

        {/* Recent Activity */}
        <View style={styles.activityHeader}>
          <Text style={[styles.sectionTitle, { color: colors.foreground }]}>Recent Activity</Text>
          <Pressable>
            <Text style={[styles.viewAll, { color: '#00D4FF' }]}>View All</Text>
          </Pressable>
        </View>
        <View style={[styles.activityList, { backgroundColor: colors.surface, borderColor: colors.border }]}>
          {recentAlerts.length > 0 ? (
            recentAlerts.map((alert, index) => (
              <ActivityItem
                key={index}
                icon={alert.icon as keyof typeof MaterialIcons.glyphMap}
                title={alert.title}
                description={alert.description}
                time={alert.time}
                status={alert.status as 'success' | 'warning' | 'error' | 'info'}
              />
            ))
          ) : (
            <>
              <ActivityItem
                icon="check-circle"
                title="P2 Phase Complete"
                description="Model cluster deployment successful"
                time="2m ago"
                status="success"
              />
              <ActivityItem
                icon="sync"
                title="Agent Scaling"
                description="Qwen-Builder spawned for build tasks"
                time="5m ago"
                status="info"
              />
              <ActivityItem
                icon="memory"
                title="GPU Allocation"
                description="ripple-gpu-01 allocated to LocalAI"
                time="12m ago"
                status="info"
              />
            </>
          )}
        </View>
      </ScrollView>
    </ScreenContainer>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    padding: 16,
  },
  header: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 20,
  },
  menuButton: {
    width: 40,
    height: 40,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
    marginRight: 12,
  },
  headerContent: {
    flex: 1,
  },
  title: {
    fontSize: 28,
    fontWeight: '700',
  },
  subtitle: {
    fontSize: 14,
    marginTop: 2,
  },
  headerRight: {
    alignItems: 'flex-end',
  },
  date: {
    fontSize: 12,
  },
  time: {
    fontSize: 18,
    fontWeight: '600',
  },
  statusBanner: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: 16,
    borderRadius: 12,
    borderWidth: 1,
    marginBottom: 16,
  },
  statusBannerLeft: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
  },
  statusIndicator: {
    width: 10,
    height: 10,
    borderRadius: 5,
  },
  statusBannerTitle: {
    fontSize: 16,
    fontWeight: '600',
  },
  statusBannerValue: {
    fontSize: 14,
  },
  buildProgress: {
    padding: 16,
    borderRadius: 12,
    borderWidth: 1,
    marginBottom: 24,
  },
  buildProgressHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginBottom: 12,
  },
  buildProgressLabel: {
    fontSize: 14,
  },
  buildProgressPhase: {
    fontSize: 14,
    fontWeight: '600',
  },
  progressBar: {
    height: 8,
    borderRadius: 4,
    overflow: 'hidden',
  },
  progressFill: {
    height: '100%',
    backgroundColor: '#00D4FF',
    borderRadius: 4,
  },
  buildPhaseName: {
    fontSize: 12,
    marginTop: 8,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: '600',
    marginBottom: 16,
  },
  cardsGrid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 12,
    marginBottom: 24,
  },
  statusCard: {
    width: '48%',
    padding: 16,
    borderRadius: 12,
    borderWidth: 1,
  },
  statusCardHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 12,
  },
  statusDot: {
    width: 8,
    height: 8,
    borderRadius: 4,
  },
  statusValue: {
    fontSize: 32,
    fontWeight: '700',
  },
  statusTitle: {
    fontSize: 14,
    marginTop: 4,
  },
  statusSubtitle: {
    fontSize: 12,
    marginTop: 2,
  },
  quickActionsRow: {
    flexDirection: 'row',
    gap: 12,
    marginBottom: 24,
  },
  quickAction: {
    flex: 1,
    padding: 16,
    borderRadius: 12,
    borderWidth: 1,
    alignItems: 'center',
    gap: 8,
  },
  quickActionIcon: {
    width: 48,
    height: 48,
    borderRadius: 24,
    alignItems: 'center',
    justifyContent: 'center',
  },
  quickActionLabel: {
    fontSize: 12,
    fontWeight: '500',
  },
  activityHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 16,
  },
  viewAll: {
    fontSize: 14,
    fontWeight: '500',
  },
  activityList: {
    borderRadius: 12,
    borderWidth: 1,
    overflow: 'hidden',
    marginBottom: 32,
  },
  activityItem: {
    flexDirection: 'row',
    alignItems: 'center',
    padding: 16,
    borderBottomWidth: 1,
    gap: 12,
  },
  activityIcon: {
    width: 32,
    height: 32,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
  },
  activityContent: {
    flex: 1,
  },
  activityTitle: {
    fontSize: 14,
    fontWeight: '500',
  },
  activityDescription: {
    fontSize: 12,
    marginTop: 2,
  },
  activityTime: {
    fontSize: 12,
  },
});
