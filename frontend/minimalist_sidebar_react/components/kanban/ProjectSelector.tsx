// Project selector dropdown component
import React, { useState } from 'react';
import { View, Text, Pressable, Modal, FlatList, Platform } from 'react-native';
import type { Project } from '@/shared/vibe-kanban/types';

interface ProjectSelectorProps {
  projects: Project[];
  selectedId: string | null;
  onSelect: (projectId: string) => void;
}

export function ProjectSelector({ projects, selectedId, onSelect }: ProjectSelectorProps) {
  const [isOpen, setIsOpen] = useState(false);
  const selectedProject = projects.find(p => p.id === selectedId);

  const handleSelect = (projectId: string) => {
    onSelect(projectId);
    setIsOpen(false);
  };

  return (
    <View style={{ flex: 1, maxWidth: 300 }}>
      <Pressable
        onPress={() => setIsOpen(true)}
        style={({ pressed }) => ({
          flexDirection: 'row',
          alignItems: 'center',
          justifyContent: 'space-between',
          backgroundColor: pressed ? '#333333' : '#242424',
          borderWidth: 1,
          borderColor: 'rgba(255, 255, 255, 0.1)',
          borderRadius: 8,
          paddingHorizontal: 12,
          paddingVertical: 10,
        })}
      >
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
          <Text style={{ fontSize: 16 }}>📁</Text>
          <Text style={{ color: '#ffffff', fontSize: 14, fontWeight: '500' }} numberOfLines={1}>
            {selectedProject?.name || 'Select Project'}
          </Text>
        </View>
        <Text style={{ color: '#9ca3af', fontSize: 12 }}>▼</Text>
      </Pressable>

      <Modal
        visible={isOpen}
        animationType="fade"
        transparent
        onRequestClose={() => setIsOpen(false)}
      >
        <Pressable
          style={{
            flex: 1,
            backgroundColor: 'rgba(0, 0, 0, 0.5)',
            alignItems: 'center',
            justifyContent: 'center',
            padding: 20,
          }}
          onPress={() => setIsOpen(false)}
        >
          <View
            style={{
              backgroundColor: '#1a1a1a',
              borderRadius: 12,
              width: '100%',
              maxWidth: 400,
              maxHeight: '60%',
            }}
          >
            {/* Header */}
            <View
              style={{
                flexDirection: 'row',
                alignItems: 'center',
                justifyContent: 'space-between',
                padding: 16,
                borderBottomWidth: 1,
                borderBottomColor: 'rgba(255, 255, 255, 0.1)',
              }}
            >
              <Text style={{ color: '#ffffff', fontSize: 16, fontWeight: '600' }}>
                Select Project
              </Text>
              <Pressable
                onPress={() => setIsOpen(false)}
                style={{
                  width: 28,
                  height: 28,
                  borderRadius: 14,
                  backgroundColor: 'rgba(255, 255, 255, 0.1)',
                  alignItems: 'center',
                  justifyContent: 'center',
                }}
              >
                <Text style={{ color: '#9ca3af' }}>×</Text>
              </Pressable>
            </View>

            {/* Project List */}
            {projects.length === 0 ? (
              <View style={{ padding: 32, alignItems: 'center' }}>
                <Text style={{ color: '#9ca3af', fontSize: 14, marginBottom: 4 }}>
                  No projects found
                </Text>
                <Text style={{ color: '#6b7280', fontSize: 12, textAlign: 'center' }}>
                  Create a project in vibe-kanban to get started
                </Text>
              </View>
            ) : (
              <FlatList
                data={projects}
                keyExtractor={item => item.id}
                renderItem={({ item }) => (
                  <Pressable
                    onPress={() => handleSelect(item.id)}
                    style={({ pressed }) => ({
                      flexDirection: 'row',
                      alignItems: 'center',
                      gap: 12,
                      padding: 16,
                      backgroundColor: 
                        selectedId === item.id 
                          ? 'rgba(59, 130, 246, 0.1)' 
                          : pressed 
                            ? 'rgba(255, 255, 255, 0.05)' 
                            : 'transparent',
                      borderLeftWidth: 3,
                      borderLeftColor: selectedId === item.id ? '#3b82f6' : 'transparent',
                    })}
                  >
                    <Text style={{ fontSize: 20 }}>📁</Text>
                    <View style={{ flex: 1 }}>
                      <Text style={{ color: '#ffffff', fontSize: 14, fontWeight: '500' }}>
                        {item.name}
                      </Text>
                      <Text style={{ color: '#6b7280', fontSize: 12, marginTop: 2 }}>
                        Created {new Date(item.created_at).toLocaleDateString()}
                      </Text>
                    </View>
                    {selectedId === item.id && (
                      <Text style={{ color: '#3b82f6', fontSize: 16 }}>✓</Text>
                    )}
                  </Pressable>
                )}
                ItemSeparatorComponent={() => (
                  <View style={{ height: 1, backgroundColor: 'rgba(255, 255, 255, 0.05)' }} />
                )}
              />
            )}
          </View>
        </Pressable>
      </Modal>
    </View>
  );
}
