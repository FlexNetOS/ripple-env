import React, { useState } from 'react';
import { View, Text, Pressable, StyleSheet, ScrollView, Modal } from 'react-native';
import { useSidebar } from '@/contexts/sidebar-context';
import MaterialIcons from '@expo/vector-icons/MaterialIcons';

interface FavoriteItem {
  id: string;
  label: string;
  section: string;
  iconName: keyof typeof MaterialIcons.glyphMap;
  badgeCount?: number;
  badgeStatus?: 'success' | 'warning' | 'error' | 'info';
}

const defaultFavorites: FavoriteItem[] = [
  { 
    id: 'fav-1', 
    label: 'Build Pipeline', 
    section: 'build', 
    iconName: 'build',
    badgeCount: 8,
    badgeStatus: 'success',
  },
  { 
    id: 'fav-2', 
    label: 'Active Agents', 
    section: 'agents', 
    iconName: 'psychology',
    badgeCount: 5,
  },
  { 
    id: 'fav-3', 
    label: 'Infrastructure', 
    section: 'infra', 
    iconName: 'hub',
  },
  { 
    id: 'fav-4', 
    label: 'AI Command Center', 
    section: 'ai', 
    iconName: 'smart-toy',
    badgeCount: 3,
    badgeStatus: 'info',
  },
];

export function FavoritesPanel() {
  const { showFavorites, setShowFavorites, setActiveSection } = useSidebar();
  const [favorites, setFavorites] = useState<FavoriteItem[]>(defaultFavorites);

  const removeFavorite = (id: string) => {
    setFavorites(prev => prev.filter(f => f.id !== id));
  };

  const navigateToSection = (section: string) => {
    setActiveSection(section);
    setShowFavorites(false);
  };

  const statusColors: Record<string, string> = {
    success: '#22c55e',
    warning: '#f59e0b',
    error: '#ef4444',
    info: '#3b82f6',
  };

  return (
    <Modal
      visible={showFavorites}
      transparent
      animationType="fade"
      onRequestClose={() => setShowFavorites(false)}
    >
      <Pressable 
        style={styles.overlay}
        onPress={() => setShowFavorites(false)}
      >
        <Pressable 
          style={styles.panel}
          onPress={(e) => e.stopPropagation()}
        >
          {/* Header */}
          <View style={styles.header}>
            <View style={styles.headerTitle}>
              <MaterialIcons name="star" size={20} color="#facc15" />
              <Text style={styles.title}>Favorites</Text>
            </View>
            <Pressable
              onPress={() => setShowFavorites(false)}
              style={({ pressed }) => [
                styles.closeButton,
                pressed && { opacity: 0.7 },
              ]}
            >
              <MaterialIcons name="close" size={18} color="#9CA3AF" />
            </Pressable>
          </View>

          <Text style={styles.subtitle}>Quick access to your most used sections</Text>

          {/* Favorites List */}
          <ScrollView style={styles.list} showsVerticalScrollIndicator={false}>
            {favorites.length === 0 ? (
              <View style={styles.emptyState}>
                <MaterialIcons name="star-border" size={48} color="#4B5563" />
                <Text style={styles.emptyText}>No favorites yet</Text>
                <Text style={styles.emptyHint}>Star items to add them here</Text>
              </View>
            ) : (
              favorites.map((favorite) => (
                <Pressable
                  key={favorite.id}
                  style={({ pressed }) => [
                    styles.favoriteItem,
                    pressed && { backgroundColor: '#262626' },
                  ]}
                  onPress={() => navigateToSection(favorite.section)}
                >
                  <View style={styles.favoriteContent}>
                    <View style={styles.iconContainer}>
                      <MaterialIcons 
                        name={favorite.iconName} 
                        size={18} 
                        color="#FAFAFA" 
                      />
                      {favorite.badgeCount !== undefined && (
                        <View style={[
                          styles.badge,
                          { backgroundColor: favorite.badgeStatus ? statusColors[favorite.badgeStatus] : '#ef4444' }
                        ]}>
                          <Text style={styles.badgeText}>{favorite.badgeCount}</Text>
                        </View>
                      )}
                    </View>
                    <Text style={styles.favoriteLabel}>{favorite.label}</Text>
                    <Pressable
                      onPress={(e) => {
                        e.stopPropagation();
                        removeFavorite(favorite.id);
                      }}
                      style={({ pressed }) => [
                        styles.removeButton,
                        pressed && { opacity: 0.7 },
                      ]}
                    >
                      <MaterialIcons name="close" size={14} color="#6B7280" />
                    </Pressable>
                  </View>
                </Pressable>
              ))
            )}
          </ScrollView>

          {/* Footer */}
          <View style={styles.footer}>
            <Text style={styles.footerText}>
              Right-click any section to add to favorites
            </Text>
          </View>
        </Pressable>
      </Pressable>
    </Modal>
  );
}

const styles = StyleSheet.create({
  overlay: {
    flex: 1,
    backgroundColor: 'rgba(0, 0, 0, 0.5)',
    justifyContent: 'flex-start',
    alignItems: 'flex-end',
    paddingTop: 60,
    paddingRight: 16,
  },
  panel: {
    width: 320,
    maxHeight: '70%',
    backgroundColor: '#171717',
    borderRadius: 16,
    borderWidth: 1,
    borderColor: '#262626',
    overflow: 'hidden',
  },
  header: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: 16,
    borderBottomWidth: 1,
    borderBottomColor: '#262626',
  },
  headerTitle: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  title: {
    fontSize: 16,
    fontWeight: '600',
    color: '#FAFAFA',
  },
  closeButton: {
    padding: 6,
    borderRadius: 8,
  },
  subtitle: {
    fontSize: 12,
    color: '#6B7280',
    paddingHorizontal: 16,
    paddingTop: 12,
    paddingBottom: 8,
  },
  list: {
    flex: 1,
    padding: 8,
  },
  emptyState: {
    alignItems: 'center',
    justifyContent: 'center',
    paddingVertical: 48,
  },
  emptyText: {
    fontSize: 14,
    color: '#6B7280',
    marginTop: 12,
  },
  emptyHint: {
    fontSize: 12,
    color: '#4B5563',
    marginTop: 4,
  },
  favoriteItem: {
    padding: 12,
    marginBottom: 4,
    borderRadius: 10,
    backgroundColor: '#1f1f1f',
  },
  favoriteContent: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
  },
  iconContainer: {
    position: 'relative',
  },
  badge: {
    position: 'absolute',
    top: -6,
    right: -8,
    minWidth: 16,
    height: 16,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
    paddingHorizontal: 4,
    borderWidth: 2,
    borderColor: '#1f1f1f',
  },
  badgeText: {
    fontSize: 9,
    fontWeight: '600',
    color: '#FFFFFF',
  },
  favoriteLabel: {
    flex: 1,
    fontSize: 14,
    fontWeight: '500',
    color: '#FAFAFA',
  },
  removeButton: {
    padding: 4,
  },
  footer: {
    padding: 12,
    borderTopWidth: 1,
    borderTopColor: '#262626',
  },
  footerText: {
    fontSize: 11,
    color: '#4B5563',
    textAlign: 'center',
  },
});

export default FavoritesPanel;
