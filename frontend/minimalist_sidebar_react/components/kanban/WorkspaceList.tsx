// Workspace list component showing task attempts
import React from 'react';
import { View, Text, Pressable, FlatList } from 'react-native';
import type { WorkspaceWithStatus } from '@/shared/vibe-kanban/types';

interface WorkspaceListProps {
  workspaces: WorkspaceWithStatus[];
  selectedWorkspaceId: string | null;
  onSelectWorkspace: (workspace: WorkspaceWithStatus) => void;
}

export function WorkspaceList({ 
  workspaces, 
  selectedWorkspaceId, 
  onSelectWorkspace 
}: WorkspaceListProps) {
  if (workspaces.length === 0) {
    return (
      <View
        style={{
          backgroundColor: '#242424',
          borderRadius: 8,
          padding: 16,
          alignItems: 'center',
        }}
      >
        <Text style={{ color: '#9ca3af', fontSize: 14, marginBottom: 4 }}>
          No workspaces yet
        </Text>
        <Text style={{ color: '#6b7280', fontSize: 12, textAlign: 'center' }}>
          Start an agent to create a workspace
        </Text>
      </View>
    );
  }

  const renderWorkspace = ({ item: workspace }: { item: WorkspaceWithStatus }) => {
    const isSelected = selectedWorkspaceId === workspace.id;
    const statusColor = workspace.is_running 
      ? '#3b82f6' 
      : workspace.is_errored 
        ? '#ef4444' 
        : '#22c55e';
    const statusLabel = workspace.is_running 
      ? 'Running' 
      : workspace.is_errored 
        ? 'Error' 
        : 'Completed';

    return (
      <Pressable
        onPress={() => onSelectWorkspace(workspace)}
        style={({ pressed }) => ({
          backgroundColor: isSelected ? 'rgba(59, 130, 246, 0.1)' : '#242424',
          borderWidth: 1,
          borderColor: isSelected ? '#3b82f6' : 'rgba(255, 255, 255, 0.1)',
          borderRadius: 8,
          padding: 12,
          marginBottom: 8,
          opacity: pressed ? 0.8 : 1,
        })}
      >
        <View style={{ flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center' }}>
          <View style={{ flex: 1 }}>
            <Text style={{ color: '#ffffff', fontSize: 14, fontWeight: '500' }}>
              {workspace.name || `Workspace ${workspace.branch}`}
            </Text>
            <Text style={{ color: '#9ca3af', fontSize: 12, marginTop: 2 }}>
              Branch: {workspace.branch}
            </Text>
          </View>
          
          <View style={{ alignItems: 'flex-end' }}>
            <View
              style={{
                flexDirection: 'row',
                alignItems: 'center',
                gap: 6,
                backgroundColor: `${statusColor}20`,
                paddingHorizontal: 8,
                paddingVertical: 4,
                borderRadius: 12,
              }}
            >
              <View
                style={{
                  width: 6,
                  height: 6,
                  borderRadius: 3,
                  backgroundColor: statusColor,
                }}
              />
              <Text style={{ color: statusColor, fontSize: 11, fontWeight: '500' }}>
                {statusLabel}
              </Text>
            </View>
            
            <View style={{ flexDirection: 'row', gap: 8, marginTop: 4 }}>
              {workspace.pinned && (
                <Text style={{ color: '#f59e0b', fontSize: 10 }}>📌 Pinned</Text>
              )}
              {workspace.archived && (
                <Text style={{ color: '#6b7280', fontSize: 10 }}>📦 Archived</Text>
              )}
            </View>
          </View>
        </View>
      </Pressable>
    );
  };

  return (
    <FlatList
      data={workspaces}
      renderItem={renderWorkspace}
      keyExtractor={item => item.id}
      scrollEnabled={false}
    />
  );
}
