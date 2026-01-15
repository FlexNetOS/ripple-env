// Create task modal component
import React, { useState } from 'react';
import { View, Text, TextInput, Pressable, Modal, ScrollView, Platform } from 'react-native';
import { useCreateTask } from '@/hooks/vibe-kanban';
import { AgentSelector } from './AgentSelector';
import type { TaskStatus, BaseCodingAgent } from '@/shared/vibe-kanban/types';

interface CreateTaskModalProps {
  visible: boolean;
  projectId: string | null;
  onClose: () => void;
  onCreated?: () => void;
}

export function CreateTaskModal({ visible, projectId, onClose, onCreated }: CreateTaskModalProps) {
  const createTask = useCreateTask();
  
  const [title, setTitle] = useState('');
  const [description, setDescription] = useState('');
  const [status, setStatus] = useState<TaskStatus>('todo');
  const [selectedAgent, setSelectedAgent] = useState<BaseCodingAgent>('CLAUDE_CODE');
  const [startImmediately, setStartImmediately] = useState(false);

  const handleCreate = async () => {
    if (!title.trim() || !projectId) return;
    
    try {
      await createTask.mutateAsync({
        project_id: projectId,
        title: title.trim(),
        description: description.trim() || null,
        status,
        parent_workspace_id: null,
        image_ids: null,
        shared_task_id: null,
      });
      
      // Reset form
      setTitle('');
      setDescription('');
      setStatus('todo');
      setStartImmediately(false);
      
      onCreated?.();
      onClose();
    } catch (error) {
      console.error('Failed to create task:', error);
    }
  };

  const handleClose = () => {
    setTitle('');
    setDescription('');
    setStatus('todo');
    setStartImmediately(false);
    onClose();
  };

  const statusOptions: { value: TaskStatus; label: string }[] = [
    { value: 'todo', label: 'To Do' },
    { value: 'inprogress', label: 'In Progress' },
    { value: 'inreview', label: 'In Review' },
    { value: 'done', label: 'Done' },
  ];

  return (
    <Modal
      visible={visible}
      animationType="fade"
      transparent
      onRequestClose={handleClose}
    >
      <View
        style={{
          flex: 1,
          backgroundColor: 'rgba(0, 0, 0, 0.8)',
          alignItems: 'center',
          justifyContent: 'center',
          padding: 20,
        }}
      >
        <View
          style={{
            backgroundColor: '#1a1a1a',
            borderRadius: 16,
            width: '100%',
            maxWidth: 500,
            maxHeight: '90%',
            ...(Platform.OS === 'web' ? { maxHeight: '80vh' } : {}),
          }}
        >
          {/* Header */}
          <View
            style={{
              flexDirection: 'row',
              alignItems: 'center',
              justifyContent: 'space-between',
              padding: 20,
              borderBottomWidth: 1,
              borderBottomColor: 'rgba(255, 255, 255, 0.1)',
            }}
          >
            <Text style={{ color: '#ffffff', fontSize: 18, fontWeight: '600' }}>
              Create New Task
            </Text>
            <Pressable
              onPress={handleClose}
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

          {/* Form */}
          <ScrollView style={{ flex: 1 }} contentContainerStyle={{ padding: 20, gap: 20 }}>
            {/* Title */}
            <View>
              <Text style={{ color: '#9ca3af', fontSize: 12, marginBottom: 8, textTransform: 'uppercase', letterSpacing: 0.5 }}>
                Title *
              </Text>
              <TextInput
                value={title}
                onChangeText={setTitle}
                placeholder="Enter task title"
                placeholderTextColor="#6b7280"
                style={{
                  backgroundColor: '#242424',
                  borderWidth: 1,
                  borderColor: 'rgba(255, 255, 255, 0.1)',
                  borderRadius: 8,
                  padding: 12,
                  color: '#ffffff',
                  fontSize: 14,
                }}
              />
            </View>

            {/* Description */}
            <View>
              <Text style={{ color: '#9ca3af', fontSize: 12, marginBottom: 8, textTransform: 'uppercase', letterSpacing: 0.5 }}>
                Description
              </Text>
              <TextInput
                value={description}
                onChangeText={setDescription}
                placeholder="Describe what needs to be done..."
                placeholderTextColor="#6b7280"
                multiline
                numberOfLines={4}
                style={{
                  backgroundColor: '#242424',
                  borderWidth: 1,
                  borderColor: 'rgba(255, 255, 255, 0.1)',
                  borderRadius: 8,
                  padding: 12,
                  color: '#ffffff',
                  fontSize: 14,
                  minHeight: 100,
                  textAlignVertical: 'top',
                }}
              />
            </View>

            {/* Initial Status */}
            <View>
              <Text style={{ color: '#9ca3af', fontSize: 12, marginBottom: 8, textTransform: 'uppercase', letterSpacing: 0.5 }}>
                Initial Status
              </Text>
              <View style={{ flexDirection: 'row', flexWrap: 'wrap', gap: 8 }}>
                {statusOptions.map(option => (
                  <Pressable
                    key={option.value}
                    onPress={() => setStatus(option.value)}
                    style={{
                      paddingHorizontal: 16,
                      paddingVertical: 8,
                      borderRadius: 8,
                      backgroundColor: status === option.value ? '#3b82f6' : 'rgba(255, 255, 255, 0.05)',
                      borderWidth: 1,
                      borderColor: status === option.value ? '#3b82f6' : 'rgba(255, 255, 255, 0.1)',
                    }}
                  >
                    <Text style={{ color: status === option.value ? '#ffffff' : '#9ca3af', fontSize: 13 }}>
                      {option.label}
                    </Text>
                  </Pressable>
                ))}
              </View>
            </View>

            {/* Agent Selector */}
            <AgentSelector
              selectedAgent={selectedAgent}
              onSelectAgent={setSelectedAgent}
            />

            {/* Start Immediately Toggle */}
            <Pressable
              onPress={() => setStartImmediately(!startImmediately)}
              style={{
                flexDirection: 'row',
                alignItems: 'center',
                gap: 12,
                padding: 12,
                backgroundColor: startImmediately ? 'rgba(59, 130, 246, 0.1)' : 'rgba(255, 255, 255, 0.05)',
                borderRadius: 8,
                borderWidth: 1,
                borderColor: startImmediately ? '#3b82f6' : 'rgba(255, 255, 255, 0.1)',
              }}
            >
              <View
                style={{
                  width: 20,
                  height: 20,
                  borderRadius: 4,
                  borderWidth: 2,
                  borderColor: startImmediately ? '#3b82f6' : '#6b7280',
                  backgroundColor: startImmediately ? '#3b82f6' : 'transparent',
                  alignItems: 'center',
                  justifyContent: 'center',
                }}
              >
                {startImmediately && (
                  <Text style={{ color: '#ffffff', fontSize: 12, fontWeight: 'bold' }}>✓</Text>
                )}
              </View>
              <View style={{ flex: 1 }}>
                <Text style={{ color: '#ffffff', fontSize: 14, fontWeight: '500' }}>
                  Start agent immediately
                </Text>
                <Text style={{ color: '#9ca3af', fontSize: 12, marginTop: 2 }}>
                  Create workspace and begin work right away
                </Text>
              </View>
            </Pressable>
          </ScrollView>

          {/* Footer */}
          <View
            style={{
              flexDirection: 'row',
              gap: 12,
              padding: 20,
              borderTopWidth: 1,
              borderTopColor: 'rgba(255, 255, 255, 0.1)',
            }}
          >
            <Pressable
              onPress={handleClose}
              style={({ pressed }) => ({
                flex: 1,
                backgroundColor: pressed ? '#333333' : '#242424',
                paddingVertical: 14,
                borderRadius: 8,
                alignItems: 'center',
              })}
            >
              <Text style={{ color: '#9ca3af', fontWeight: '500', fontSize: 15 }}>
                Cancel
              </Text>
            </Pressable>
            <Pressable
              onPress={handleCreate}
              disabled={!title.trim() || !projectId || createTask.isPending}
              style={({ pressed }) => ({
                flex: 1,
                backgroundColor: !title.trim() || !projectId ? 'rgba(59, 130, 246, 0.3)' : pressed ? '#2563eb' : '#3b82f6',
                paddingVertical: 14,
                borderRadius: 8,
                alignItems: 'center',
                opacity: createTask.isPending ? 0.7 : 1,
              })}
            >
              <Text style={{ color: '#ffffff', fontWeight: '600', fontSize: 15 }}>
                {createTask.isPending ? 'Creating...' : 'Create Task'}
              </Text>
            </Pressable>
          </View>
        </View>
      </View>
    </Modal>
  );
}
