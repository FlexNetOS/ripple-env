// Task card component for kanban board
import React from 'react';
import { View, Text, Pressable, Platform } from 'react-native';
import type { TaskWithAttemptStatus, TaskStatus } from '@/shared/vibe-kanban/types';

interface TaskCardProps {
  task: TaskWithAttemptStatus;
  onPress: () => void;
  isDragging?: boolean;
}

const STATUS_COLORS: Record<TaskStatus, string> = {
  todo: '#94a3b8',
  inprogress: '#3b82f6',
  inreview: '#f59e0b',
  done: '#22c55e',
  cancelled: '#ef4444',
};

const EXECUTOR_ICONS: Record<string, string> = {
  CLAUDE_CODE: '🤖',
  CODEX: '🔮',
  GEMINI_CLI: '✨',
  CUSTOM: '⚙️',
};

export function TaskCard({ task, onPress, isDragging }: TaskCardProps) {
  const executorIcon = EXECUTOR_ICONS[task.executor] || '🔧';
  
  return (
    <Pressable
      onPress={onPress}
      style={({ pressed }) => ({
        backgroundColor: pressed ? '#2a2a2a' : '#242424',
        borderRadius: 8,
        padding: 12,
        marginBottom: 8,
        borderLeftWidth: 3,
        borderLeftColor: task.has_in_progress_attempt 
          ? '#3b82f6' 
          : task.last_attempt_failed 
            ? '#ef4444' 
            : STATUS_COLORS[task.status],
        opacity: isDragging ? 0.5 : 1,
        transform: [{ scale: isDragging ? 1.02 : 1 }],
        ...(Platform.OS === 'web' ? {
          cursor: 'pointer',
          transition: 'all 0.15s ease',
        } : {}),
        shadowColor: '#000',
        shadowOffset: { width: 0, height: 2 },
        shadowOpacity: isDragging ? 0.3 : 0.1,
        shadowRadius: isDragging ? 8 : 4,
        elevation: isDragging ? 8 : 2,
      })}
    >
      {/* Title */}
      <Text
        style={{
          color: '#ffffff',
          fontSize: 14,
          fontWeight: '500',
          marginBottom: task.description ? 4 : 0,
        }}
        numberOfLines={2}
      >
        {task.title}
      </Text>

      {/* Description */}
      {task.description && (
        <Text
          style={{
            color: '#9ca3af',
            fontSize: 12,
            marginBottom: 8,
          }}
          numberOfLines={2}
        >
          {task.description}
        </Text>
      )}

      {/* Footer */}
      <View
        style={{
          flexDirection: 'row',
          alignItems: 'center',
          justifyContent: 'space-between',
          marginTop: 8,
        }}
      >
        {/* Status badges */}
        <View style={{ flexDirection: 'row', gap: 6, flexWrap: 'wrap' }}>
          {task.has_in_progress_attempt && (
            <View
              style={{
                backgroundColor: 'rgba(59, 130, 246, 0.2)',
                paddingHorizontal: 8,
                paddingVertical: 3,
                borderRadius: 12,
                flexDirection: 'row',
                alignItems: 'center',
                gap: 4,
              }}
            >
              <View
                style={{
                  width: 6,
                  height: 6,
                  borderRadius: 3,
                  backgroundColor: '#3b82f6',
                }}
              />
              <Text style={{ color: '#3b82f6', fontSize: 10, fontWeight: '500' }}>
                Running
              </Text>
            </View>
          )}
          {task.last_attempt_failed && (
            <View
              style={{
                backgroundColor: 'rgba(239, 68, 68, 0.2)',
                paddingHorizontal: 8,
                paddingVertical: 3,
                borderRadius: 12,
              }}
            >
              <Text style={{ color: '#ef4444', fontSize: 10, fontWeight: '500' }}>
                Failed
              </Text>
            </View>
          )}
        </View>

        {/* Executor badge */}
        <View
          style={{
            backgroundColor: 'rgba(255, 255, 255, 0.1)',
            paddingHorizontal: 6,
            paddingVertical: 2,
            borderRadius: 4,
            flexDirection: 'row',
            alignItems: 'center',
            gap: 4,
          }}
        >
          <Text style={{ fontSize: 10 }}>{executorIcon}</Text>
          <Text style={{ color: '#9ca3af', fontSize: 10 }}>
            {task.executor?.replace('_', ' ').toLowerCase() || 'unassigned'}
          </Text>
        </View>
      </View>
    </Pressable>
  );
}
