// Task details panel with tabbed interface
import React, { useState } from 'react';
import { View, Text, ScrollView, Pressable, TextInput, ActivityIndicator } from 'react-native';
import { useTask, useWorkspaces, useStartWorkspace, useStopWorkspace, useFollowUp } from '@/hooks/vibe-kanban';
import { AgentSelector } from './AgentSelector';
import { WorkspaceList } from './WorkspaceList';
import { LogViewer } from '../logs/LogViewer';
import { DiffViewer } from '../diff/DiffViewer';
import type { TaskWithAttemptStatus, BaseCodingAgent, WorkspaceWithStatus } from '@/shared/vibe-kanban/types';

type TabType = 'details' | 'logs' | 'diffs';

interface TaskDetailsPanelProps {
  taskId: string;
  task?: TaskWithAttemptStatus;
  onClose: () => void;
}

export function TaskDetailsPanel({ taskId, task: initialTask, onClose }: TaskDetailsPanelProps) {
  const { data: task, isLoading: taskLoading } = useTask(taskId);
  const { data: workspaces, isLoading: workspacesLoading } = useWorkspaces(taskId);
  const startWorkspace = useStartWorkspace();
  const stopWorkspace = useStopWorkspace();
  const followUp = useFollowUp();
  
  const [activeTab, setActiveTab] = useState<TabType>('details');
  const [selectedAgent, setSelectedAgent] = useState<BaseCodingAgent>('CLAUDE_CODE');
  const [selectedWorkspace, setSelectedWorkspace] = useState<WorkspaceWithStatus | null>(null);
  const [followUpPrompt, setFollowUpPrompt] = useState('');

  const taskData = task || initialTask;
  const activeWorkspace = workspaces?.find(w => w.is_running);

  const handleStartAgent = async () => {
    if (!selectedWorkspace && workspaces && workspaces.length > 0) {
      // Use first available workspace or create one
      const workspace = workspaces[0];
      await startWorkspace.mutateAsync({ 
        workspaceId: workspace.id, 
        executorProfileId: selectedAgent 
      });
    }
  };

  const handleStopAgent = async () => {
    if (activeWorkspace) {
      await stopWorkspace.mutateAsync(activeWorkspace.id);
    }
  };

  const handleFollowUp = async () => {
    if (!activeWorkspace || !followUpPrompt.trim()) return;
    await followUp.mutateAsync({
      workspaceId: activeWorkspace.id,
      data: {
        prompt: followUpPrompt,
        variant: null,
        retry_process_id: null,
        force_when_dirty: null,
        perform_git_reset: null,
      },
    });
    setFollowUpPrompt('');
  };

  if (taskLoading || !taskData) {
    return (
      <View style={{ flex: 1, backgroundColor: '#0a0a0a', alignItems: 'center', justifyContent: 'center' }}>
        <ActivityIndicator color="#3b82f6" />
      </View>
    );
  }

  return (
    <View style={{ flex: 1, backgroundColor: '#0a0a0a' }}>
      {/* Header */}
      <View style={{ padding: 16, borderBottomWidth: 1, borderBottomColor: 'rgba(255, 255, 255, 0.1)' }}>
        <View style={{ flexDirection: 'row', alignItems: 'flex-start', justifyContent: 'space-between' }}>
          <View style={{ flex: 1, marginRight: 12 }}>
            <Text style={{ color: '#ffffff', fontSize: 18, fontWeight: '600' }} numberOfLines={2}>
              {taskData.title}
            </Text>
            <Text style={{ color: '#9ca3af', fontSize: 12, marginTop: 4 }}>
              ID: {taskId.slice(0, 8)}...
            </Text>
          </View>
          <Pressable
            onPress={onClose}
            style={{
              width: 32,
              height: 32,
              borderRadius: 16,
              backgroundColor: 'rgba(255, 255, 255, 0.1)',
              alignItems: 'center',
              justifyContent: 'center',
            }}
          >
            <Text style={{ color: '#9ca3af', fontSize: 18 }}>×</Text>
          </Pressable>
        </View>

        {/* Tab Bar */}
        <View style={{ flexDirection: 'row', marginTop: 16, gap: 8 }}>
          {(['details', 'logs', 'diffs'] as TabType[]).map(tab => (
            <Pressable
              key={tab}
              onPress={() => setActiveTab(tab)}
              style={{
                paddingHorizontal: 16,
                paddingVertical: 8,
                borderRadius: 8,
                backgroundColor: activeTab === tab ? '#3b82f6' : 'rgba(255, 255, 255, 0.05)',
              }}
            >
              <Text
                style={{
                  color: activeTab === tab ? '#ffffff' : '#9ca3af',
                  fontSize: 14,
                  fontWeight: '500',
                  textTransform: 'capitalize',
                }}
              >
                {tab}
              </Text>
            </Pressable>
          ))}
        </View>
      </View>

      {/* Content */}
      <ScrollView style={{ flex: 1 }} contentContainerStyle={{ padding: 16 }}>
        {activeTab === 'details' && (
          <View style={{ gap: 20 }}>
            {/* Description */}
            <View>
              <Text style={{ color: '#9ca3af', fontSize: 12, textTransform: 'uppercase', letterSpacing: 0.5, marginBottom: 8 }}>
                Description
              </Text>
              <Text style={{ color: '#ffffff', fontSize: 14, lineHeight: 20 }}>
                {taskData.description || 'No description provided'}
              </Text>
            </View>

            {/* Agent Selector */}
            <AgentSelector
              selectedAgent={selectedAgent}
              onSelectAgent={setSelectedAgent}
              disabled={!!activeWorkspace}
            />

            {/* Workspaces */}
            <View>
              <Text style={{ color: '#9ca3af', fontSize: 12, textTransform: 'uppercase', letterSpacing: 0.5, marginBottom: 8 }}>
                Workspaces ({workspaces?.length || 0})
              </Text>
              {workspacesLoading ? (
                <ActivityIndicator color="#3b82f6" />
              ) : (
                <WorkspaceList
                  workspaces={workspaces || []}
                  selectedWorkspaceId={selectedWorkspace?.id || null}
                  onSelectWorkspace={setSelectedWorkspace}
                />
              )}
            </View>
          </View>
        )}

        {activeTab === 'logs' && (
          <LogViewer
            workspaceId={selectedWorkspace?.id || activeWorkspace?.id || null}
          />
        )}

        {activeTab === 'diffs' && (
          <DiffViewer
            workspaceId={selectedWorkspace?.id || activeWorkspace?.id || null}
            repoId={null} // Will be selected from workspace repos
          />
        )}
      </ScrollView>

      {/* Follow-up Input */}
      {activeWorkspace && (
        <View style={{ padding: 16, borderTopWidth: 1, borderTopColor: 'rgba(255, 255, 255, 0.1)' }}>
          <View style={{ flexDirection: 'row', gap: 8 }}>
            <TextInput
              value={followUpPrompt}
              onChangeText={setFollowUpPrompt}
              placeholder="Send follow-up prompt..."
              placeholderTextColor="#6b7280"
              multiline
              style={{
                flex: 1,
                backgroundColor: '#1a1a1a',
                borderWidth: 1,
                borderColor: 'rgba(255, 255, 255, 0.1)',
                borderRadius: 8,
                padding: 12,
                color: '#ffffff',
                fontSize: 14,
                maxHeight: 100,
              }}
            />
            <Pressable
              onPress={handleFollowUp}
              disabled={!followUpPrompt.trim() || followUp.isPending}
              style={{
                backgroundColor: followUpPrompt.trim() ? '#3b82f6' : 'rgba(59, 130, 246, 0.3)',
                paddingHorizontal: 16,
                paddingVertical: 12,
                borderRadius: 8,
                alignItems: 'center',
                justifyContent: 'center',
              }}
            >
              <Text style={{ color: '#ffffff', fontWeight: '500' }}>
                {followUp.isPending ? '...' : 'Send'}
              </Text>
            </Pressable>
          </View>
        </View>
      )}

      {/* Action Bar */}
      <View style={{ padding: 16, borderTopWidth: 1, borderTopColor: 'rgba(255, 255, 255, 0.1)' }}>
        {activeWorkspace ? (
          <Pressable
            onPress={handleStopAgent}
            disabled={stopWorkspace.isPending}
            style={({ pressed }) => ({
              backgroundColor: pressed ? '#dc2626' : '#ef4444',
              paddingVertical: 14,
              borderRadius: 8,
              alignItems: 'center',
              opacity: stopWorkspace.isPending ? 0.7 : 1,
            })}
          >
            <Text style={{ color: '#ffffff', fontWeight: '600', fontSize: 15 }}>
              {stopWorkspace.isPending ? 'Stopping...' : 'Stop Agent'}
            </Text>
          </Pressable>
        ) : (
          <Pressable
            onPress={handleStartAgent}
            disabled={startWorkspace.isPending || !selectedAgent}
            style={({ pressed }) => ({
              backgroundColor: pressed ? '#2563eb' : '#3b82f6',
              paddingVertical: 14,
              borderRadius: 8,
              alignItems: 'center',
              opacity: startWorkspace.isPending || !selectedAgent ? 0.7 : 1,
            })}
          >
            <Text style={{ color: '#ffffff', fontWeight: '600', fontSize: 15 }}>
              {startWorkspace.isPending ? 'Starting...' : 'Start Agent'}
            </Text>
          </Pressable>
        )}
      </View>
    </View>
  );
}
