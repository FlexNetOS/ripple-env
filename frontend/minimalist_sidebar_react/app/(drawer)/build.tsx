import React from 'react';
import { ScrollView, Text, View, Pressable, StyleSheet } from 'react-native';
import { ScreenContainer } from '@/components/screen-container';
import { useColors } from '@/hooks/use-colors';
import { DrawerActions, useNavigation } from '@react-navigation/native';
import MaterialIcons from '@expo/vector-icons/MaterialIcons';
import Animated, { useAnimatedStyle, withRepeat, withTiming, withSequence } from 'react-native-reanimated';

const buildPhases = [
  { id: 'P0', name: 'Environment Foundation', status: 'completed', description: 'NixOS, Pixi, Nushell setup' },
  { id: 'P1', name: 'Vendor Integration', status: 'completed', description: 'AGiXT, LocalAI, Holochain' },
  { id: 'P2', name: 'Model Cluster', status: 'completed', description: 'DeepSeek, Qwen, Gemma deployment' },
  { id: 'P3', name: 'Inference & State', status: 'running', description: 'LocalAI + state management', progress: 45 },
  { id: 'P4', name: 'Database & Inference', status: 'pending', description: 'Qdrant, PostgreSQL, Redis' },
  { id: 'P5', name: 'Orchestration', status: 'pending', description: 'MOE policy, agent routing' },
  { id: 'P6', name: 'Testing & Promotion', status: 'pending', description: 'Integration tests, staging' },
  { id: 'P7', name: 'Build & Package', status: 'pending', description: 'Production artifacts' },
];

function PhaseCard({ phase }: { phase: typeof buildPhases[0] }) {
  const colors = useColors();
  
  const statusConfig = {
    completed: { color: '#00E676', icon: 'check-circle' as const, bg: 'rgba(0, 230, 118, 0.1)' },
    running: { color: '#00D4FF', icon: 'sync' as const, bg: 'rgba(0, 212, 255, 0.1)' },
    pending: { color: colors.muted, icon: 'schedule' as const, bg: 'transparent' },
    failed: { color: '#FF5252', icon: 'error' as const, bg: 'rgba(255, 82, 82, 0.1)' },
  };

  const config = statusConfig[phase.status as keyof typeof statusConfig];

  const spinStyle = useAnimatedStyle(() => {
    if (phase.status !== 'running') return {};
    return {
      transform: [{
        rotate: withRepeat(
          withTiming('360deg', { duration: 2000 }),
          -1,
          false
        ),
      }],
    };
  });

  return (
    <View style={[styles.phaseCard, { backgroundColor: config.bg, borderColor: config.color }]}>
      <View style={styles.phaseHeader}>
        <View style={styles.phaseIdContainer}>
          <Text style={[styles.phaseId, { color: config.color }]}>{phase.id}</Text>
        </View>
        <Animated.View style={spinStyle}>
          <MaterialIcons name={config.icon} size={24} color={config.color} />
        </Animated.View>
      </View>
      <Text style={[styles.phaseName, { color: colors.foreground }]}>{phase.name}</Text>
      <Text style={[styles.phaseDescription, { color: colors.muted }]}>{phase.description}</Text>
      {phase.progress !== undefined && (
        <View style={styles.progressContainer}>
          <View style={[styles.progressBar, { backgroundColor: colors.border }]}>
            <View style={[styles.progressFill, { width: `${phase.progress}%` }]} />
          </View>
          <Text style={[styles.progressText, { color: config.color }]}>{phase.progress}%</Text>
        </View>
      )}
    </View>
  );
}

export default function BuildScreen() {
  const colors = useColors();
  const navigation = useNavigation();

  const completedPhases = buildPhases.filter(p => p.status === 'completed').length;
  const totalPhases = buildPhases.length;
  const overallProgress = Math.round((completedPhases / totalPhases) * 100);

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
            <Text style={[styles.title, { color: colors.foreground }]}>Build Pipeline</Text>
            <Text style={[styles.subtitle, { color: colors.muted }]}>P0-P7 Phase Execution</Text>
          </View>
        </View>

        {/* Overall Progress */}
        <View style={[styles.overallProgress, { backgroundColor: colors.surface, borderColor: colors.border }]}>
          <View style={styles.overallHeader}>
            <Text style={[styles.overallTitle, { color: colors.foreground }]}>Overall Progress</Text>
            <Text style={[styles.overallPercent, { color: '#00D4FF' }]}>{overallProgress}%</Text>
          </View>
          <View style={[styles.progressBar, { backgroundColor: colors.border }]}>
            <View style={[styles.progressFill, { width: `${overallProgress}%` }]} />
          </View>
          <Text style={[styles.overallSubtext, { color: colors.muted }]}>
            {completedPhases} of {totalPhases} phases completed
          </Text>
        </View>

        {/* Quick Actions */}
        <View style={styles.quickActions}>
          <Pressable style={[styles.actionButton, { backgroundColor: '#00D4FF' }]}>
            <MaterialIcons name="play-arrow" size={20} color="#000000" />
            <Text style={styles.actionButtonText}>Resume Build</Text>
          </Pressable>
          <Pressable style={[styles.actionButton, { backgroundColor: colors.surface, borderColor: colors.border, borderWidth: 1 }]}>
            <MaterialIcons name="description" size={20} color={colors.foreground} />
            <Text style={[styles.actionButtonText, { color: colors.foreground }]}>View Logs</Text>
          </Pressable>
        </View>

        {/* Phase Cards */}
        <Text style={[styles.sectionTitle, { color: colors.foreground }]}>Build Phases</Text>
        <View style={styles.phasesContainer}>
          {buildPhases.map((phase) => (
            <PhaseCard key={phase.id} phase={phase} />
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
  overallProgress: {
    padding: 20,
    borderRadius: 16,
    borderWidth: 1,
    marginBottom: 20,
  },
  overallHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 12,
  },
  overallTitle: {
    fontSize: 16,
    fontWeight: '600',
  },
  overallPercent: {
    fontSize: 24,
    fontWeight: '700',
  },
  overallSubtext: {
    fontSize: 12,
    marginTop: 8,
  },
  quickActions: {
    flexDirection: 'row',
    gap: 12,
    marginBottom: 24,
  },
  actionButton: {
    flex: 1,
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    gap: 8,
    paddingVertical: 14,
    borderRadius: 12,
  },
  actionButtonText: {
    fontSize: 14,
    fontWeight: '600',
    color: '#000000',
  },
  sectionTitle: {
    fontSize: 18,
    fontWeight: '600',
    marginBottom: 16,
  },
  phasesContainer: {
    gap: 12,
    marginBottom: 32,
  },
  phaseCard: {
    padding: 16,
    borderRadius: 12,
    borderWidth: 1,
  },
  phaseHeader: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 12,
  },
  phaseIdContainer: {
    paddingHorizontal: 10,
    paddingVertical: 4,
    borderRadius: 6,
    backgroundColor: 'rgba(255, 255, 255, 0.1)',
  },
  phaseId: {
    fontSize: 12,
    fontWeight: '700',
  },
  phaseName: {
    fontSize: 16,
    fontWeight: '600',
    marginBottom: 4,
  },
  phaseDescription: {
    fontSize: 13,
  },
  progressContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
    marginTop: 12,
  },
  progressBar: {
    flex: 1,
    height: 6,
    borderRadius: 3,
    overflow: 'hidden',
  },
  progressFill: {
    height: '100%',
    backgroundColor: '#00D4FF',
    borderRadius: 3,
  },
  progressText: {
    fontSize: 12,
    fontWeight: '600',
    minWidth: 36,
  },
});
