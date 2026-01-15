// Kanban board component with drag-and-drop support
import React, { useMemo } from 'react';
import { View, ScrollView, Platform, Text } from 'react-native';
import { KanbanColumn } from './KanbanColumn';
import { TaskCard } from './TaskCard';
import type { TaskWithAttemptStatus, TaskStatus } from '@/shared/vibe-kanban/types';

interface KanbanBoardProps {
  tasks: TaskWithAttemptStatus[];
  onTaskMove: (taskId: string, newStatus: TaskStatus) => void;
  onTaskSelect: (task: TaskWithAttemptStatus) => void;
  isLoading?: boolean;
}

const COLUMNS: TaskStatus[] = ['todo', 'inprogress', 'inreview', 'done'];

export function KanbanBoard({ 
  tasks, 
  onTaskMove, 
  onTaskSelect,
  isLoading,
}: KanbanBoardProps) {
  // Group tasks by status
  const tasksByStatus = useMemo(() => {
    return COLUMNS.reduce((acc, status) => {
      acc[status] = tasks.filter(t => t.status === status);
      return acc;
    }, {} as Record<TaskStatus, TaskWithAttemptStatus[]>);
  }, [tasks]);

  // Web-based drag handling (simplified - can use dnd-kit for full implementation)
  const handleDragStart = (e: React.DragEvent, taskId: string) => {
    if (Platform.OS !== 'web') return;
    e.dataTransfer.setData('taskId', taskId);
    e.dataTransfer.effectAllowed = 'move';
  };

  const handleDragOver = (e: React.DragEvent) => {
    if (Platform.OS !== 'web') return;
    e.preventDefault();
    e.dataTransfer.dropEffect = 'move';
  };

  const handleDrop = (e: React.DragEvent, status: TaskStatus) => {
    if (Platform.OS !== 'web') return;
    e.preventDefault();
    const taskId = e.dataTransfer.getData('taskId');
    if (taskId) {
      onTaskMove(taskId, status);
    }
  };

  if (isLoading) {
    return (
      <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center' }}>
        <Text style={{ color: '#9ca3af' }}>Loading tasks...</Text>
      </View>
    );
  }

  if (tasks.length === 0) {
    return (
      <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', padding: 40 }}>
        <Text style={{ color: '#9ca3af', fontSize: 16, marginBottom: 8 }}>No tasks yet</Text>
        <Text style={{ color: '#6b7280', fontSize: 14, textAlign: 'center' }}>
          Create a new task to get started with your project
        </Text>
      </View>
    );
  }

  return (
    <ScrollView
      horizontal
      showsHorizontalScrollIndicator={false}
      contentContainerStyle={{
        padding: 16,
        gap: 16,
        flexDirection: 'row',
        minHeight: '100%',
      }}
      style={{ flex: 1, backgroundColor: '#0a0a0a' }}
    >
      {COLUMNS.map(status => (
        <View
          key={status}
          // @ts-ignore - Web-specific props
          onDragOver={Platform.OS === 'web' ? handleDragOver : undefined}
          onDrop={Platform.OS === 'web' ? (e: any) => handleDrop(e, status) : undefined}
          style={{ height: '100%' }}
        >
          <KanbanColumn status={status} count={tasksByStatus[status].length}>
            {tasksByStatus[status].map(task => (
              <View
                key={task.id}
                // @ts-ignore - Web-specific props
                draggable={Platform.OS === 'web'}
                onDragStart={Platform.OS === 'web' ? (e: any) => handleDragStart(e, task.id) : undefined}
              >
                <TaskCard
                  task={task}
                  onPress={() => onTaskSelect(task)}
                />
              </View>
            ))}
          </KanbanColumn>
        </View>
      ))}
    </ScrollView>
  );
}
