import React from 'react';
import { ScrollView, Text, View, Pressable, StyleSheet } from 'react-native';
import { ScreenContainer } from '@/components/screen-container';
import { useColors } from '@/hooks/use-colors';
import { DrawerActions, useNavigation } from '@react-navigation/native';
import MaterialIcons from '@expo/vector-icons/MaterialIcons';

const nodes = [
  { id: '1', name: 'ripple-master', type: 'Master', status: 'healthy', cpu: 45, memory: 62, storage: 38, gpu: null },
  { id: '2', name: 'ripple-worker-01', type: 'Worker', status: 'healthy', cpu: 78, memory: 85, storage: 45, gpu: null },
  { id: '3', name: 'ripple-worker-02', type: 'Worker', status: 'healthy', cpu: 56, memory: 71, storage: 52, gpu: null },
  { id: '4', name: 'ripple-gpu-01', type: 'GPU', status: 'healthy', cpu: 34, memory: 48, storage: 25, gpu: 92 },
];

const services = [
  { id: '1', name: 'Holochain Conductor', status: 'running', port: 8888, uptime: '7d 12h' },
  { id: '2', name: 'Qdrant Vector DB', status: 'running', port: 6333, uptime: '7d 12h' },
  { id: '3', name: 'Redis Cache', status: 'running', port: 6379, uptime: '7d 12h' },
  { id: '4', name: 'PostgreSQL', status: 'running', port: 5432, uptime: '7d 12h' },
  { id: '5', name: 'LocalAI Server', status: 'running', port: 8080, uptime: '3d 5h' },
];

function ResourceBar({ value, label, warning = 80 }: { value: number; label: string; warning?: number }) {
  const colors = useColors();
  const barColor = value > warning ? '#FFB300' : value > 90 ? '#FF5252' : '#00E676';

  return (
    <View style={styles.resourceItem}>
      <View style={styles.resourceHeader}>
        <Text style={[styles.resourceLabel, { color: colors.muted }]}>{label}</Text>
        <Text style={[styles.resourceValue, { color: barColor }]}>{value}%</Text>
      </View>
      <View style={[styles.resourceBar, { backgroundColor: colors.border }]}>
        <View style={[styles.resourceFill, { width: `${value}%`, backgroundColor: barColor }]} />
      </View>
    </View>
  );
}

function NodeCard({ node }: { node: typeof nodes[0] }) {
  const colors = useColors();
  
  const statusConfig = {
    healthy: { color: '#00E676', icon: 'check-circle' as const },
    warning: { color: '#FFB300', icon: 'warning' as const },
    error: { color: '#FF5252', icon: 'error' as const },
  };

  const config = statusConfig[node.status as keyof typeof statusConfig];

  const typeIcon: Record<string, keyof typeof MaterialIcons.glyphMap> = {
    Master: 'dns',
    Worker: 'memory',
    GPU: 'developer-board',
  };

  return (
    <View style={[styles.nodeCard, { backgroundColor: colors.surface, borderColor: colors.border }]}>
      <View style={styles.nodeHeader}>
        <View style={[styles.nodeIcon, { backgroundColor: `${config.color}20` }]}>
          <MaterialIcons name={typeIcon[node.type] || 'dns'} size={24} color={config.color} />
        </View>
        <View style={styles.nodeInfo}>
          <Text style={[styles.nodeName, { color: colors.foreground }]}>{node.name}</Text>
          <Text style={[styles.nodeType, { color: colors.muted }]}>{node.type} Node</Text>
        </View>
        <MaterialIcons name={config.icon} size={20} color={config.color} />
      </View>
      
      <View style={styles.nodeResources}>
        <ResourceBar value={node.cpu} label="CPU" />
        <ResourceBar value={node.memory} label="Memory" />
        <ResourceBar value={node.storage} label="Storage" />
        {node.gpu !== null && <ResourceBar value={node.gpu} label="GPU" />}
      </View>
    </View>
  );
}

function ServiceRow({ service }: { service: typeof services[0] }) {
  const colors = useColors();
  
  const statusConfig = {
    running: { color: '#00E676', label: 'Running' },
    stopped: { color: '#666666', label: 'Stopped' },
    error: { color: '#FF5252', label: 'Error' },
  };

  const config = statusConfig[service.status as keyof typeof statusConfig];

  return (
    <View style={[styles.serviceRow, { borderBottomColor: colors.border }]}>
      <View style={styles.serviceInfo}>
        <View style={[styles.statusIndicator, { backgroundColor: config.color }]} />
        <View>
          <Text style={[styles.serviceName, { color: colors.foreground }]}>{service.name}</Text>
          <Text style={[styles.servicePort, { color: colors.muted }]}>Port {service.port}</Text>
        </View>
      </View>
      <View style={styles.serviceStats}>
        <Text style={[styles.serviceUptime, { color: colors.muted }]}>{service.uptime}</Text>
        <Text style={[styles.serviceStatus, { color: config.color }]}>{config.label}</Text>
      </View>
    </View>
  );
}

export default function InfraScreen() {
  const colors = useColors();
  const navigation = useNavigation();

  const healthyNodes = nodes.filter(n => n.status === 'healthy').length;
  const runningServices = services.filter(s => s.status === 'running').length;

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
            <Text style={[styles.title, { color: colors.foreground }]}>Infrastructure</Text>
            <Text style={[styles.subtitle, { color: colors.muted }]}>Nodes & Services</Text>
          </View>
        </View>

        {/* Summary */}
        <View style={styles.summaryRow}>
          <View style={[styles.summaryCard, { backgroundColor: 'rgba(0, 230, 118, 0.1)', borderColor: '#00E676' }]}>
            <MaterialIcons name="dns" size={24} color="#00E676" />
            <Text style={[styles.summaryValue, { color: '#00E676' }]}>{healthyNodes}/{nodes.length}</Text>
            <Text style={[styles.summaryLabel, { color: colors.muted }]}>Nodes Healthy</Text>
          </View>
          <View style={[styles.summaryCard, { backgroundColor: 'rgba(0, 212, 255, 0.1)', borderColor: '#00D4FF' }]}>
            <MaterialIcons name="settings-applications" size={24} color="#00D4FF" />
            <Text style={[styles.summaryValue, { color: '#00D4FF' }]}>{runningServices}/{services.length}</Text>
            <Text style={[styles.summaryLabel, { color: colors.muted }]}>Services Running</Text>
          </View>
        </View>

        {/* Nodes */}
        <Text style={[styles.sectionTitle, { color: colors.foreground }]}>Compute Nodes</Text>
        <View style={styles.nodesList}>
          {nodes.map((node) => (
            <NodeCard key={node.id} node={node} />
          ))}
        </View>

        {/* Services */}
        <Text style={[styles.sectionTitle, { color: colors.foreground }]}>Services</Text>
        <View style={[styles.servicesList, { backgroundColor: colors.surface, borderColor: colors.border }]}>
          {services.map((service) => (
            <ServiceRow key={service.id} service={service} />
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
    gap: 8,
  },
  summaryValue: {
    fontSize: 24,
    fontWeight: '700',
  },
  summaryLabel: {
    fontSize: 12,
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: '600',
    marginBottom: 16,
  },
  nodesList: {
    gap: 12,
    marginBottom: 24,
  },
  nodeCard: {
    padding: 16,
    borderRadius: 12,
    borderWidth: 1,
  },
  nodeHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 16,
  },
  nodeIcon: {
    width: 48,
    height: 48,
    borderRadius: 12,
    alignItems: 'center',
    justifyContent: 'center',
  },
  nodeInfo: {
    flex: 1,
    marginLeft: 12,
  },
  nodeName: {
    fontSize: 16,
    fontWeight: '600',
  },
  nodeType: {
    fontSize: 13,
    marginTop: 2,
  },
  nodeResources: {
    gap: 12,
  },
  resourceItem: {
    gap: 4,
  },
  resourceHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  resourceLabel: {
    fontSize: 12,
  },
  resourceValue: {
    fontSize: 12,
    fontWeight: '600',
  },
  resourceBar: {
    height: 6,
    borderRadius: 3,
    overflow: 'hidden',
  },
  resourceFill: {
    height: '100%',
    borderRadius: 3,
  },
  servicesList: {
    borderRadius: 12,
    borderWidth: 1,
    overflow: 'hidden',
    marginBottom: 32,
  },
  serviceRow: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: 16,
    borderBottomWidth: 1,
  },
  serviceInfo: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
  },
  statusIndicator: {
    width: 8,
    height: 8,
    borderRadius: 4,
  },
  serviceName: {
    fontSize: 14,
    fontWeight: '500',
  },
  servicePort: {
    fontSize: 12,
    marginTop: 2,
  },
  serviceStats: {
    alignItems: 'flex-end',
  },
  serviceUptime: {
    fontSize: 12,
  },
  serviceStatus: {
    fontSize: 12,
    fontWeight: '500',
    marginTop: 2,
  },
});
