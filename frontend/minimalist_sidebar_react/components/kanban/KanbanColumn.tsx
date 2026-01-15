// Kanban column component for task grouping
import React from 'react';
import { View, Text, ScrollView, Platform } from 'react-native';
import type { TaskStatus } from '@/shared/vibe-kanban/types';

interface KanbanColumnProps {
  status: TaskStatus;
  count: number;
  children: React.ReactNode;
}

const STATUS_CONFIG: Record<TaskStatus, { label: string; color: string; bgColor: string }> = {
  todo: {
    label: 'To Do',
    color: '#94a3b8',
    bgColor: 'rgba(148, 163, 184, 0.1)',
  },
  inprogress: {
    label: 'In Progress',
    color: '#3b82f6',
    bgColor: 'rgba(59, 130, 246, 0.1)',
  },
  inreview: {
    label: 'In Review',
    color: '#f59e0b',
    bgColor: 'rgba(245, 158, 11, 0.1)',
  },
  done: {
    label: 'Done',
    color: '#22c55e',
    bgColor: 'rgba(34, 197, 94, 0.1)',
  },
  cancelled: {
    label: 'Cancelled',
    color: '#ef4444',
    bgColor: 'rgba(239, 68, 68, 0.1)',
  },
};

export function KanbanColumn({ status, count, children }: KanbanColumnProps) {
  const config = STATUS_CONFIG[status];
  
  return (
    <View
      style={{
        width: Platform.OS === 'web' ? 320 : 280,
        backgroundColor: '#1a1a1a',
        borderRadius: 12,
        overflow: 'hidden',
      }}
    >
      {/* Column Header */}
      <View
        style={{
          flexDirection: 'row',
          alignItems: 'center',
          justifyContent: 'space-between',
          padding: 12,
          backgroundColor: config.bgColor,
          borderBottomWidth: 1,
          borderBottomColor: 'rgba(255, 255, 255, 0.1)',
        }}
      >
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
          <View
            style={{
              width: 8,
              height: 8,
              borderRadius: 4,
              backgroundColor: config.color,
            }}
          />
          <Text
            style={{
              color: '#ffffff',
              fontSize: 14,
              fontWeight: '600',
            }}
          >
            {config.label}
          </Text>
        </View>
        <View
          style={{
            backgroundColor: 'rgba(255, 255, 255, 0.1)',
            paddingHorizontal: 8,
            paddingVertical: 2,
            borderRadius: 10,
          }}
        >
          <Text
            style={{
              color: '#9ca3af',
              fontSize: 12,
              fontWeight: '500',
            }}
          >
            {count}
          </Text>
        </View>
      </View>

      {/* Column Content */}
      <ScrollView
        style={{ flex: 1, padding: 8 }}
        showsVerticalScrollIndicator={false}
      >
        {children}
        {/* Drop zone indicator */}
        <View
          style={{
            height: 60,
            borderRadius: 8,
            borderWidth: 2,
            borderStyle: 'dashed',
            borderColor: 'rgba(255, 255, 255, 0.1)',
            marginTop: 8,
            alignItems: 'center',
            justifyContent: 'center',
          }}
        >
          <Text style={{ color: 'rgba(255, 255, 255, 0.3)', fontSize: 12 }}>
            Drop here
          </Text>
        </View>
      </ScrollView>
    </View>
  );
}

export { STATUS_CONFIG };
