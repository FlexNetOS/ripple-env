import React from 'react';
import { ScrollView, Text, View, Pressable, StyleSheet } from 'react-native';
import { ScreenContainer } from '@/components/screen-container';
import { useColors } from '@/hooks/use-colors';
import { DrawerActions, useNavigation } from '@react-navigation/native';
import MaterialIcons from '@expo/vector-icons/MaterialIcons';

const agents = [
  { id: '1', name: 'AGiXT-01', type: 'Orchestrator', status: 'online', tasks: 12, cpu: 45, memory: 62 },
  { id: '2', name: 'LocalAI-Primary', type: 'Inference', status: 'online', tasks: 8, cpu: 78, memory: 85 },
  { id: '3', name: 'DeepSeek-R1', type: 'Reasoning', status: 'online', tasks: 5, cpu: 56, memory: 71 },
  { id: '4', name: 'Qwen-Builder', type: 'Builder', status: 'busy', tasks: 3, cpu: 92, memory: 88 },
  { id: '5', name: 'Gemma-Validator', type: 'Validator', status: 'online', tasks: 2, cpu: 23, memory: 45 },
];

const moePolicy = {
  routingStrategy: 'Load Balanced',
  fallbackChain: ['AGiXT-01', 'LocalAI-Primary', 'DeepSeek-R1'],
  autoScaling: { min: 3, max: 10, trigger: '80% CPU' },
};

function AgentCard({ agent }: { agent: typeof agents[0] }) {
  const colors = useColors();
  
  const statusConfig = {
    online: { color: '#00E676', label: 'Online' },
    offline: { color: '#666666', label: 'Offline' },
    busy: { color: '#FFB300', label: 'Busy' },
  };

  const config = statusConfig[agent.status as keyof typeof statusConfig];

  const iconMap: Record<string, keyof typeof MaterialIcons.glyphMap> = {
    Orchestrator: 'smart-toy',
    Inference: 'memory',
    Reasoning: 'psychology',
    Builder: 'build',
    Validator: 'verified',
  };

  return (
    <View style={[styles.agentCard, { backgroundColor: colors.surface, borderColor: colors.border }]}>
      <View style={styles.agentHeader}>
        <View style={[styles.agentIcon, { backgroundColor: `${config.color}20` }]}>
          <MaterialIcons name={iconMap[agent.type] || 'smart-toy'} size={24} color={config.color} />
        </View>
        <View style={styles.agentInfo}>
          <Text style={[styles.agentName, { color: colors.foreground }]}>{agent.name}</Text>
          <Text style={[styles.agentType, { color: colors.muted }]}>{agent.type}</Text>
        </View>
        <View style={styles.statusContainer}>
          <View style={[styles.statusDot, { backgroundColor: config.color }]} />
          <Text style={[styles.statusText, { color: config.color }]}>{config.label}</Text>
        </View>
      </View>
      
      <View style={styles.agentStats}>
        <View style={styles.statItem}>
          <Text style={[styles.statValue, { color: colors.foreground }]}>{agent.tasks}</Text>
          <Text style={[styles.statLabel, { color: colors.muted }]}>Tasks</Text>
        </View>
        <View style={styles.statItem}>
          <Text style={[styles.statValue, { color: agent.cpu > 80 ? '#FFB300' : colors.foreground }]}>{agent.cpu}%</Text>
          <Text style={[styles.statLabel, { color: colors.muted }]}>CPU</Text>
        </View>
        <View style={styles.statItem}>
          <Text style={[styles.statValue, { color: agent.memory > 80 ? '#FFB300' : colors.foreground }]}>{agent.memory}%</Text>
          <Text style={[styles.statLabel, { color: colors.muted }]}>Memory</Text>
        </View>
      </View>
    </View>
  );
}

function PolicyCard() {
  const colors = useColors();

  return (
    <View style={[styles.policyCard, { backgroundColor: colors.surface, borderColor: colors.border }]}>
      <View style={styles.policyHeader}>
        <MaterialIcons name="account-tree" size={20} color="#9B7BFF" />
        <Text style={[styles.policyTitle, { color: colors.foreground }]}>MOE Policy</Text>
      </View>
      
      <View style={styles.policyItem}>
        <Text style={[styles.policyLabel, { color: colors.muted }]}>Routing Strategy</Text>
        <Text style={[styles.policyValue, { color: colors.foreground }]}>{moePolicy.routingStrategy}</Text>
      </View>
      
      <View style={styles.policyItem}>
        <Text style={[styles.policyLabel, { color: colors.muted }]}>Fallback Chain</Text>
        <View style={styles.fallbackChain}>
          {moePolicy.fallbackChain.map((agent, index) => (
            <View key={agent} style={styles.fallbackItem}>
              <Text style={[styles.fallbackText, { color: colors.foreground }]}>{agent}</Text>
              {index < moePolicy.fallbackChain.length - 1 && (
                <MaterialIcons name="chevron-right" size={16} color={colors.muted} />
              )}
            </View>
          ))}
        </View>
      </View>
      
      <View style={styles.policyItem}>
        <Text style={[styles.policyLabel, { color: colors.muted }]}>Auto-Scaling</Text>
        <Text style={[styles.policyValue, { color: colors.foreground }]}>
          {moePolicy.autoScaling.min}-{moePolicy.autoScaling.max} agents @ {moePolicy.autoScaling.trigger}
        </Text>
      </View>
    </View>
  );
}

export default function AgentsScreen() {
  const colors = useColors();
  const navigation = useNavigation();

  const onlineCount = agents.filter(a => a.status === 'online').length;
  const busyCount = agents.filter(a => a.status === 'busy').length;

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
            <Text style={[styles.title, { color: colors.foreground }]}>Agents</Text>
            <Text style={[styles.subtitle, { color: colors.muted }]}>MOE Agent Management</Text>
          </View>
        </View>

        {/* Summary */}
        <View style={styles.summaryRow}>
          <View style={[styles.summaryCard, { backgroundColor: 'rgba(0, 230, 118, 0.1)', borderColor: '#00E676' }]}>
            <Text style={[styles.summaryValue, { color: '#00E676' }]}>{onlineCount}</Text>
            <Text style={[styles.summaryLabel, { color: colors.muted }]}>Online</Text>
          </View>
          <View style={[styles.summaryCard, { backgroundColor: 'rgba(255, 179, 0, 0.1)', borderColor: '#FFB300' }]}>
            <Text style={[styles.summaryValue, { color: '#FFB300' }]}>{busyCount}</Text>
            <Text style={[styles.summaryLabel, { color: colors.muted }]}>Busy</Text>
          </View>
          <View style={[styles.summaryCard, { backgroundColor: 'rgba(0, 212, 255, 0.1)', borderColor: '#00D4FF' }]}>
            <Text style={[styles.summaryValue, { color: '#00D4FF' }]}>{agents.length}</Text>
            <Text style={[styles.summaryLabel, { color: colors.muted }]}>Total</Text>
          </View>
        </View>

        {/* MOE Policy */}
        <Text style={[styles.sectionTitle, { color: colors.foreground }]}>Routing Policy</Text>
        <PolicyCard />

        {/* Agent List */}
        <Text style={[styles.sectionTitle, { color: colors.foreground }]}>Active Agents</Text>
        <View style={styles.agentsList}>
          {agents.map((agent) => (
            <AgentCard key={agent.id} agent={agent} />
          ))}
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
  summaryRow: {
    flexDirection: 'row',
    gap: 12,
    marginBottom: 24,
  },
  summaryCard: {
    flex: 1,
    padding: 16,
    borderRadius: 12,
    borderWidth: 1,
    alignItems: 'center',
  },
  summaryValue: {
    fontSize: 28,
    fontWeight: '700',
  },
  summaryLabel: {
    fontSize: 12,
    marginTop: 4,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: '600',
    marginBottom: 16,
  },
  policyCard: {
    padding: 16,
    borderRadius: 12,
    borderWidth: 1,
    marginBottom: 24,
  },
  policyHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
    marginBottom: 16,
  },
  policyTitle: {
    fontSize: 16,
    fontWeight: '600',
  },
  policyItem: {
    marginBottom: 12,
  },
  policyLabel: {
    fontSize: 12,
    marginBottom: 4,
  },
  policyValue: {
    fontSize: 14,
    fontWeight: '500',
  },
  fallbackChain: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    alignItems: 'center',
  },
  fallbackItem: {
    flexDirection: 'row',
    alignItems: 'center',
  },
  fallbackText: {
    fontSize: 14,
    fontWeight: '500',
  },
  agentsList: {
    gap: 12,
    marginBottom: 32,
  },
  agentCard: {
    padding: 16,
    borderRadius: 12,
    borderWidth: 1,
  },
  agentHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 16,
  },
  agentIcon: {
    width: 48,
    height: 48,
    borderRadius: 12,
    alignItems: 'center',
    justifyContent: 'center',
  },
  agentInfo: {
    flex: 1,
    marginLeft: 12,
  },
  agentName: {
    fontSize: 16,
    fontWeight: '600',
  },
  agentType: {
    fontSize: 13,
    marginTop: 2,
  },
  statusContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 6,
  },
  statusDot: {
    width: 8,
    height: 8,
    borderRadius: 4,
  },
  statusText: {
    fontSize: 12,
    fontWeight: '500',
  },
  agentStats: {
    flexDirection: 'row',
    justifyContent: 'space-around',
    paddingTop: 12,
    borderTopWidth: 1,
    borderTopColor: 'rgba(255, 255, 255, 0.1)',
  },
  statItem: {
    alignItems: 'center',
  },
  statValue: {
    fontSize: 18,
    fontWeight: '600',
  },
  statLabel: {
    fontSize: 11,
    marginTop: 2,
  },
});
