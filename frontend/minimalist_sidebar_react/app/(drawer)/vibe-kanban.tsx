import React, { useEffect, useState, useCallback } from 'react';
import { 
  View, 
  Text, 
  StyleSheet, 
  ScrollView, 
  Pressable, 
  ActivityIndicator,
  Linking,
  Platform,
  RefreshControl,
} from 'react-native';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useColors } from '@/hooks/use-colors';
import Feather from '@expo/vector-icons/Feather';
import MaterialCommunityIcons from '@expo/vector-icons/MaterialCommunityIcons';
import { 
  getVibeKanbanClient, 
  type Project, 
  type VibeKanbanConfig 
} from '@/services/vibe-kanban';

// Status indicator colors
const STATUS_COLORS = {
  online: '#22C55E',
  running: '#3B82F6',
  pending: '#F59E0B',
  completed: '#22C55E',
  offline: '#6B7280',
  error: '#EF4444',
};

interface ConnectionStatusProps {
  isConnected: boolean;
  config: VibeKanbanConfig;
}

function ConnectionStatus({ isConnected, config }: ConnectionStatusProps) {
  const colors = useColors();
  
  return (
    <View style={[styles.statusBar, { borderBottomColor: colors.border }]}>
      <View style={styles.statusIndicator}>
        <View 
          style={[
            styles.statusDot, 
            { backgroundColor: isConnected ? STATUS_COLORS.online : STATUS_COLORS.offline }
          ]} 
        />
        <Text style={[styles.statusText, { color: colors.muted }]}>
          {isConnected ? 'Connected' : 'Disconnected'}
        </Text>
      </View>
      <Text style={[styles.statusUrl, { color: colors.muted }]} numberOfLines={1}>
        {config.apiBaseUrl}
      </Text>
    </View>
  );
}

interface ProjectCardProps {
  project: Project;
  onPress: () => void;
}

function ProjectCard({ project, onPress }: ProjectCardProps) {
  const colors = useColors();
  
  return (
    <Pressable 
      style={({ pressed }) => [
        styles.projectCard,
        { 
          backgroundColor: colors.card,
          borderColor: colors.border,
          opacity: pressed ? 0.8 : 1,
        }
      ]}
      onPress={onPress}
    >
      <View style={styles.projectHeader}>
        <MaterialCommunityIcons name="folder-outline" size={20} color="#8B5CF6" />
        <Text style={[styles.projectName, { color: colors.foreground }]} numberOfLines={1}>
          {project.name}
        </Text>
      </View>
      <Text style={[styles.projectPath, { color: colors.muted }]} numberOfLines={1}>
        {project.path}
      </Text>
      <Text style={[styles.projectDate, { color: colors.muted }]}>
        Created: {new Date(project.created_at).toLocaleDateString()}
      </Text>
    </Pressable>
  );
}

interface QuickActionProps {
  icon: string;
  label: string;
  onPress: () => void;
  variant?: 'primary' | 'secondary';
}

function QuickAction({ icon, label, onPress, variant = 'secondary' }: QuickActionProps) {
  const colors = useColors();
  const isPrimary = variant === 'primary';
  
  return (
    <Pressable
      style={({ pressed }) => [
        styles.quickAction,
        { 
          backgroundColor: isPrimary ? '#8B5CF6' : colors.card,
          borderColor: isPrimary ? '#8B5CF6' : colors.border,
          opacity: pressed ? 0.8 : 1,
        }
      ]}
      onPress={onPress}
    >
      <Feather 
        name={icon as any} 
        size={18} 
        color={isPrimary ? '#FFFFFF' : colors.foreground} 
      />
      <Text 
        style={[
          styles.quickActionText, 
          { color: isPrimary ? '#FFFFFF' : colors.foreground }
        ]}
      >
        {label}
      </Text>
    </Pressable>
  );
}

export default function VibeKanbanScreen() {
  const insets = useSafeAreaInsets();
  const colors = useColors();
  const client = getVibeKanbanClient();
  
  const [isConnected, setIsConnected] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [isRefreshing, setIsRefreshing] = useState(false);
  const [projects, setProjects] = useState<Project[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [config] = useState(client.getConfig());

  const checkConnection = useCallback(async () => {
    const available = await client.isAvailable();
    setIsConnected(available);
    return available;
  }, [client]);

  const loadProjects = useCallback(async () => {
    const result = await client.getProjects();
    if (result.success && result.data) {
      setProjects(result.data);
      setError(null);
    } else {
      setError(result.error || 'Failed to load projects');
    }
  }, [client]);

  const refresh = useCallback(async () => {
    setIsRefreshing(true);
    const connected = await checkConnection();
    if (connected) {
      await loadProjects();
    }
    setIsRefreshing(false);
  }, [checkConnection, loadProjects]);

  useEffect(() => {
    async function init() {
      setIsLoading(true);
      const connected = await checkConnection();
      if (connected) {
        await loadProjects();
      }
      setIsLoading(false);
    }
    init();
  }, [checkConnection, loadProjects]);

  const openVibeKanban = useCallback(async () => {
    const url = config.frontendUrl;
    if (Platform.OS === 'web') {
      window.open(url, '_blank');
    } else {
      const canOpen = await Linking.canOpenURL(url);
      if (canOpen) {
        await Linking.openURL(url);
      }
    }
  }, [config.frontendUrl]);

  const handleProjectPress = useCallback((project: Project) => {
    // Open vibe-kanban with the project selected
    const url = `${config.frontendUrl}/project/${project.id}`;
    if (Platform.OS === 'web') {
      window.open(url, '_blank');
    } else {
      Linking.openURL(url);
    }
  }, [config.frontendUrl]);

  return (
    <View style={[styles.container, { backgroundColor: colors.background, paddingTop: insets.top }]}>
      {/* Header */}
      <View style={[styles.header, { borderBottomColor: colors.border }]}>
        <View style={styles.headerTitle}>
          <MaterialCommunityIcons name="view-column-outline" size={24} color="#8B5CF6" />
          <Text style={[styles.title, { color: colors.foreground }]}>Vibe Kanban</Text>
        </View>
        <Text style={[styles.subtitle, { color: colors.muted }]}>
          AI Coding Agent Orchestration
        </Text>
      </View>

      {/* Connection Status */}
      <ConnectionStatus isConnected={isConnected} config={config} />

      <ScrollView 
        style={styles.content}
        contentContainerStyle={styles.contentContainer}
        refreshControl={
          <RefreshControl
            refreshing={isRefreshing}
            onRefresh={refresh}
            tintColor={colors.foreground}
          />
        }
      >
        {/* Quick Actions */}
        <View style={styles.section}>
          <Text style={[styles.sectionTitle, { color: colors.muted }]}>QUICK ACTIONS</Text>
          <View style={styles.quickActionsGrid}>
            <QuickAction 
              icon="external-link" 
              label="Open Dashboard" 
              onPress={openVibeKanban}
              variant="primary"
            />
            <QuickAction 
              icon="refresh-cw" 
              label="Refresh" 
              onPress={refresh}
            />
          </View>
        </View>

        {/* Connection Info */}
        {!isConnected && !isLoading && (
          <View style={[styles.infoCard, { backgroundColor: colors.card, borderColor: colors.border }]}>
            <Feather name="info" size={20} color="#F59E0B" />
            <View style={styles.infoContent}>
              <Text style={[styles.infoTitle, { color: colors.foreground }]}>
                Vibe Kanban Not Running
              </Text>
              <Text style={[styles.infoText, { color: colors.muted }]}>
                Start vibe-kanban with `npx vibe-kanban` or configure the connection in settings.
              </Text>
            </View>
          </View>
        )}

        {/* Loading State */}
        {isLoading && (
          <View style={styles.loadingContainer}>
            <ActivityIndicator size="large" color="#8B5CF6" />
            <Text style={[styles.loadingText, { color: colors.muted }]}>
              Connecting to Vibe Kanban...
            </Text>
          </View>
        )}

        {/* Error State */}
        {error && isConnected && (
          <View style={[styles.errorCard, { backgroundColor: '#FEE2E2', borderColor: '#EF4444' }]}>
            <Feather name="alert-circle" size={20} color="#EF4444" />
            <Text style={styles.errorText}>{error}</Text>
          </View>
        )}

        {/* Projects List */}
        {isConnected && projects.length > 0 && (
          <View style={styles.section}>
            <Text style={[styles.sectionTitle, { color: colors.muted }]}>
              PROJECTS ({projects.length})
            </Text>
            {projects.map((project) => (
              <ProjectCard 
                key={project.id} 
                project={project} 
                onPress={() => handleProjectPress(project)}
              />
            ))}
          </View>
        )}

        {/* Empty State */}
        {isConnected && projects.length === 0 && !isLoading && !error && (
          <View style={[styles.emptyState, { borderColor: colors.border }]}>
            <MaterialCommunityIcons name="folder-open-outline" size={48} color={colors.muted} />
            <Text style={[styles.emptyTitle, { color: colors.foreground }]}>No Projects</Text>
            <Text style={[styles.emptyText, { color: colors.muted }]}>
              Create a new project in Vibe Kanban to get started.
            </Text>
            <Pressable 
              style={[styles.emptyButton, { backgroundColor: '#8B5CF6' }]}
              onPress={openVibeKanban}
            >
              <Text style={styles.emptyButtonText}>Open Vibe Kanban</Text>
            </Pressable>
          </View>
        )}

        {/* Embedded iframe for web (optional) */}
        {Platform.OS === 'web' && isConnected && (
          <View style={styles.section}>
            <Text style={[styles.sectionTitle, { color: colors.muted }]}>EMBEDDED VIEW</Text>
            <View style={[styles.embedContainer, { borderColor: colors.border }]}>
              <iframe
                src={config.frontendUrl}
                style={{
                  width: '100%',
                  height: 500,
                  border: 'none',
                  borderRadius: 8,
                }}
                title="Vibe Kanban"
              />
            </View>
          </View>
        )}
      </ScrollView>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
  },
  header: {
    paddingHorizontal: 20,
    paddingVertical: 16,
    borderBottomWidth: 1,
  },
  headerTitle: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 10,
  },
  title: {
    fontSize: 24,
    fontWeight: '700',
  },
  subtitle: {
    fontSize: 14,
    marginTop: 4,
    marginLeft: 34,
  },
  statusBar: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingHorizontal: 20,
    paddingVertical: 10,
    borderBottomWidth: 1,
  },
  statusIndicator: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
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
  statusUrl: {
    fontSize: 11,
    maxWidth: '50%',
  },
  content: {
    flex: 1,
  },
  contentContainer: {
    padding: 20,
    gap: 24,
  },
  section: {
    gap: 12,
  },
  sectionTitle: {
    fontSize: 11,
    fontWeight: '600',
    letterSpacing: 0.5,
  },
  quickActionsGrid: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 12,
  },
  quickAction: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
    paddingHorizontal: 16,
    paddingVertical: 12,
    borderRadius: 8,
    borderWidth: 1,
  },
  quickActionText: {
    fontSize: 14,
    fontWeight: '500',
  },
  infoCard: {
    flexDirection: 'row',
    alignItems: 'flex-start',
    gap: 12,
    padding: 16,
    borderRadius: 8,
    borderWidth: 1,
  },
  infoContent: {
    flex: 1,
  },
  infoTitle: {
    fontSize: 14,
    fontWeight: '600',
    marginBottom: 4,
  },
  infoText: {
    fontSize: 13,
    lineHeight: 18,
  },
  loadingContainer: {
    alignItems: 'center',
    justifyContent: 'center',
    paddingVertical: 40,
    gap: 16,
  },
  loadingText: {
    fontSize: 14,
  },
  errorCard: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
    padding: 16,
    borderRadius: 8,
    borderWidth: 1,
  },
  errorText: {
    flex: 1,
    fontSize: 13,
    color: '#EF4444',
  },
  projectCard: {
    padding: 16,
    borderRadius: 8,
    borderWidth: 1,
    gap: 6,
  },
  projectHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 10,
  },
  projectName: {
    flex: 1,
    fontSize: 15,
    fontWeight: '600',
  },
  projectPath: {
    fontSize: 12,
    marginLeft: 30,
  },
  projectDate: {
    fontSize: 11,
    marginLeft: 30,
  },
  emptyState: {
    alignItems: 'center',
    justifyContent: 'center',
    paddingVertical: 40,
    paddingHorizontal: 20,
    borderWidth: 1,
    borderStyle: 'dashed',
    borderRadius: 12,
    gap: 12,
  },
  emptyTitle: {
    fontSize: 16,
    fontWeight: '600',
  },
  emptyText: {
    fontSize: 14,
    textAlign: 'center',
    maxWidth: 280,
  },
  emptyButton: {
    paddingHorizontal: 20,
    paddingVertical: 10,
    borderRadius: 8,
    marginTop: 8,
  },
  emptyButtonText: {
    color: '#FFFFFF',
    fontSize: 14,
    fontWeight: '600',
  },
  embedContainer: {
    borderWidth: 1,
    borderRadius: 8,
    overflow: 'hidden',
  },
});
