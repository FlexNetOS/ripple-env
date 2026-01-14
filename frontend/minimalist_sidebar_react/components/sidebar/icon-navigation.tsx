import React, { useState } from 'react';
import { View, Text, Pressable, StyleSheet, ScrollView, Modal } from 'react-native';
import { useSidebar, type WorkspaceType } from '@/contexts/sidebar-context';
import { useColors } from '@/hooks/use-colors';
import Animated, { useAnimatedStyle, withTiming } from 'react-native-reanimated';
import MaterialIcons from '@expo/vector-icons/MaterialIcons';
import MaterialCommunityIcons from '@expo/vector-icons/MaterialCommunityIcons';
import Feather from '@expo/vector-icons/Feather';
import { ALL_PANELS, getPanelsForWorkspace, type SidebarPanel } from '@/data/sidebar-panels';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

// Complete workspace options matching Figma design
const WORKSPACES: { id: WorkspaceType; label: string; icon: string; iconLib: 'material' | 'community' | 'feather' }[] = [
  { id: 'all', label: 'All', icon: 'layers', iconLib: 'feather' },
  { id: 'productivity', label: 'Work', icon: 'briefcase', iconLib: 'feather' },
  { id: 'communication', label: 'People', icon: 'users', iconLib: 'feather' },
  { id: 'intelligence', label: 'AI', icon: 'cpu', iconLib: 'feather' },
  { id: 'infrastructure', label: 'Tech', icon: 'server', iconLib: 'feather' },
  { id: 'life', label: 'Life', icon: 'heart', iconLib: 'feather' },
  { id: 'business', label: 'Business', icon: 'briefcase', iconLib: 'feather' },
];

// Icon mapping for panels
const panelIconMap: Record<string, { lib: 'material' | 'community' | 'feather'; name: string }> = {
  'ai-command-center': { lib: 'feather', name: 'layers' },
  'life-dashboard': { lib: 'material', name: 'dashboard' },
  'contacts': { lib: 'feather', name: 'users' },
  'wallet': { lib: 'community', name: 'wallet-outline' },
  'legal-compliance': { lib: 'community', name: 'scale-balance' },
  'tasks': { lib: 'feather', name: 'check-square' },
  'projects': { lib: 'community', name: 'view-dashboard-outline' },
  'calendar': { lib: 'feather', name: 'calendar' },
  'files': { lib: 'feather', name: 'folder' },
  'analytics-hub': { lib: 'feather', name: 'bar-chart-2' },
  'social-media': { lib: 'feather', name: 'share-2' },
  'compute-nodes': { lib: 'feather', name: 'server' },
  'notifications': { lib: 'feather', name: 'bell' },
  'favorites': { lib: 'feather', name: 'star' },
  'settings': { lib: 'feather', name: 'settings' },
  'profile': { lib: 'feather', name: 'user' },
};

function renderIcon(iconId: string, size: number, color: string) {
  const mapping = panelIconMap[iconId];
  if (!mapping) {
    return <Feather name="circle" size={size} color={color} />;
  }
  
  switch (mapping.lib) {
    case 'material':
      return <MaterialIcons name={mapping.name as any} size={size} color={color} />;
    case 'community':
      return <MaterialCommunityIcons name={mapping.name as any} size={size} color={color} />;
    case 'feather':
    default:
      return <Feather name={mapping.name as any} size={size} color={color} />;
  }
}

function renderWorkspaceIcon(ws: typeof WORKSPACES[0], size: number, color: string) {
  switch (ws.iconLib) {
    case 'material':
      return <MaterialIcons name={ws.icon as any} size={size} color={color} />;
    case 'community':
      return <MaterialCommunityIcons name={ws.icon as any} size={size} color={color} />;
    case 'feather':
    default:
      return <Feather name={ws.icon as any} size={size} color={color} />;
  }
}

interface BadgeProps {
  count?: number;
  color?: string;
}

function Badge({ count, color = '#EF4444' }: BadgeProps) {
  if (count === undefined) return null;

  return (
    <View style={[styles.badge, { backgroundColor: color }]}>
      <Text style={styles.badgeText}>
        {count > 99 ? '99+' : count}
      </Text>
    </View>
  );
}

interface IconNavButtonProps {
  panel: SidebarPanel;
  isActive: boolean;
  onPress: () => void;
}

function IconNavButton({ panel, isActive, onPress }: IconNavButtonProps) {
  const colors = useColors();

  const animatedStyle = useAnimatedStyle(() => ({
    backgroundColor: withTiming(isActive ? 'rgba(0, 212, 255, 0.15)' : 'transparent', { duration: 200 }),
  }));

  return (
    <Pressable 
      onPress={onPress}
      style={({ pressed }) => [
        pressed && { opacity: 0.7 }
      ]}
    >
      <Animated.View style={[styles.iconButton, animatedStyle]}>
        {renderIcon(panel.id, 20, isActive ? '#00D4FF' : colors.muted)}
        {panel.badge !== undefined && (
          <Badge count={panel.badge} color={panel.badgeColor} />
        )}
      </Animated.View>
    </Pressable>
  );
}

export function IconNavigation() {
  const { 
    activeSection, 
    setActiveSection, 
    activeWorkspace, 
    setActiveWorkspace,
    setShowNotifications,
    setShowFavorites,
  } = useSidebar();
  const colors = useColors();
  const insets = useSafeAreaInsets();
  const [showWorkspaceMenu, setShowWorkspaceMenu] = useState(false);

  // Get panels for current workspace
  const panels = getPanelsForWorkspace(activeWorkspace as any);
  
  // Separate main panels from bottom panels
  const bottomPanelIds = ['notifications', 'favorites', 'settings', 'profile'];
  const mainPanels = panels.filter(p => !bottomPanelIds.includes(p.id));
  const bottomPanels = ALL_PANELS.filter(p => bottomPanelIds.includes(p.id));

  const currentWorkspace = WORKSPACES.find(w => w.id === activeWorkspace) || WORKSPACES[0];

  const handleWorkspaceSelect = (ws: WorkspaceType) => {
    setActiveWorkspace(ws);
    setShowWorkspaceMenu(false);
    // Select first panel of new workspace
    const newPanels = getPanelsForWorkspace(ws as any);
    if (newPanels.length > 0 && !bottomPanelIds.includes(newPanels[0].id)) {
      setActiveSection(newPanels[0].id);
    }
  };

  return (
    <View style={[styles.container, { backgroundColor: '#000000', borderRightColor: colors.border, paddingTop: insets.top + 8 }]}>
      {/* Workspace Switcher */}
      <Pressable
        onPress={() => setShowWorkspaceMenu(true)}
        style={({ pressed }) => [
          styles.workspaceSwitcher,
          pressed && { opacity: 0.7 },
        ]}
      >
        <View style={styles.workspaceIcon}>
          <Feather name="layers" size={20} color="#00D4FF" />
        </View>
        {/* Green badge */}
        <View style={styles.workspaceBadge}>
          <Text style={styles.workspaceBadgeText}>3</Text>
        </View>
      </Pressable>

      {/* Workspace Dropdown Modal */}
      <Modal
        visible={showWorkspaceMenu}
        transparent
        animationType="fade"
        onRequestClose={() => setShowWorkspaceMenu(false)}
      >
        <Pressable 
          style={styles.modalOverlay}
          onPress={() => setShowWorkspaceMenu(false)}
        >
          <View style={styles.workspaceMenu}>
            <Text style={styles.workspaceMenuTitle}>WORKSPACE</Text>
            {WORKSPACES.map((ws) => (
              <Pressable
                key={ws.id}
                style={[
                  styles.workspaceMenuItem,
                  activeWorkspace === ws.id && styles.workspaceMenuItemActive
                ]}
                onPress={() => handleWorkspaceSelect(ws.id)}
              >
                {renderWorkspaceIcon(ws, 16, activeWorkspace === ws.id ? '#00D4FF' : '#9CA3AF')}
                <Text style={[
                  styles.workspaceMenuItemText,
                  activeWorkspace === ws.id && styles.workspaceMenuItemTextActive
                ]}>
                  {ws.label}
                </Text>
              </Pressable>
            ))}
          </View>
        </Pressable>
      </Modal>

      {/* Navigation Icons - Scrollable */}
      <ScrollView 
        style={styles.navScroll}
        contentContainerStyle={styles.navContainer}
        showsVerticalScrollIndicator={false}
      >
        {mainPanels.map((panel) => (
          <IconNavButton
            key={panel.id}
            panel={panel}
            isActive={activeSection === panel.id}
            onPress={() => setActiveSection(panel.id)}
          />
        ))}
      </ScrollView>

      {/* Bottom Icons - Notifications, Favorites, Settings, Avatar */}
      <View style={styles.bottomContainer}>
        {/* Notifications */}
        <Pressable 
          style={[
            styles.iconButton,
            activeSection === 'notifications' && styles.iconButtonActive
          ]}
          onPress={() => {
            setActiveSection('notifications');
          }}
        >
          <Feather name="bell" size={20} color={activeSection === 'notifications' ? '#00D4FF' : colors.muted} />
          <Badge count={6} color="#EF4444" />
        </Pressable>
        
        {/* Favorites */}
        <Pressable 
          style={[
            styles.iconButton,
            activeSection === 'favorites' && styles.iconButtonActive
          ]}
          onPress={() => {
            setActiveSection('favorites');
          }}
        >
          <Feather name="star" size={20} color={activeSection === 'favorites' ? '#00D4FF' : colors.muted} />
          <Badge count={5} color="#3B82F6" />
        </Pressable>
        
        {/* Settings */}
        <Pressable 
          style={[
            styles.iconButton,
            activeSection === 'settings' && styles.iconButtonActive
          ]}
          onPress={() => setActiveSection('settings')}
        >
          <Feather 
            name="settings" 
            size={20} 
            color={activeSection === 'settings' ? '#00D4FF' : colors.muted} 
          />
        </Pressable>
        
        {/* Avatar/Profile */}
        <Pressable
          style={[
            styles.avatar,
            activeSection === 'profile' && styles.avatarActive
          ]}
          onPress={() => setActiveSection('profile')}
        >
          <Feather name="user" size={16} color="#FAFAFA" />
        </Pressable>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    width: 64,
    height: '100%',
    flexDirection: 'column',
    alignItems: 'center',
    paddingVertical: 16,
    borderRightWidth: 1,
  },
  workspaceSwitcher: {
    marginBottom: 12,
    position: 'relative',
  },
  workspaceIcon: {
    width: 40,
    height: 40,
    borderRadius: 12,
    alignItems: 'center',
    justifyContent: 'center',
    backgroundColor: '#1F2937',
  },
  workspaceBadge: {
    position: 'absolute',
    top: -4,
    right: -4,
    backgroundColor: '#22C55E',
    borderRadius: 8,
    minWidth: 16,
    height: 16,
    justifyContent: 'center',
    alignItems: 'center',
    paddingHorizontal: 4,
    borderWidth: 2,
    borderColor: '#000000',
  },
  workspaceBadgeText: {
    color: '#FFFFFF',
    fontSize: 10,
    fontWeight: '600',
  },
  modalOverlay: {
    flex: 1,
    backgroundColor: 'rgba(0, 0, 0, 0.5)',
    justifyContent: 'flex-start',
    paddingTop: 60,
    paddingLeft: 8,
  },
  workspaceMenu: {
    backgroundColor: '#1F2937',
    borderRadius: 8,
    padding: 8,
    width: 160,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.3,
    shadowRadius: 8,
    elevation: 8,
  },
  workspaceMenuTitle: {
    color: '#6B7280',
    fontSize: 11,
    fontWeight: '600',
    letterSpacing: 0.5,
    paddingHorizontal: 8,
    paddingVertical: 4,
    marginBottom: 4,
  },
  workspaceMenuItem: {
    flexDirection: 'row',
    alignItems: 'center',
    paddingHorizontal: 8,
    paddingVertical: 8,
    borderRadius: 6,
    gap: 8,
  },
  workspaceMenuItemActive: {
    backgroundColor: 'rgba(0, 217, 255, 0.1)',
  },
  workspaceMenuItemText: {
    color: '#9CA3AF',
    fontSize: 14,
  },
  workspaceMenuItemTextActive: {
    color: '#FFFFFF',
  },
  navScroll: {
    flex: 1,
    width: '100%',
  },
  navContainer: {
    alignItems: 'center',
    gap: 6,
    paddingVertical: 4,
  },
  bottomContainer: {
    gap: 8,
    alignItems: 'center',
    paddingTop: 12,
    borderTopWidth: 1,
    borderTopColor: '#262626',
  },
  iconButton: {
    width: 40,
    height: 40,
    borderRadius: 10,
    alignItems: 'center',
    justifyContent: 'center',
    position: 'relative',
  },
  iconButtonActive: {
    backgroundColor: 'rgba(0, 212, 255, 0.15)',
  },
  badge: {
    position: 'absolute',
    top: -4,
    right: -4,
    minWidth: 18,
    height: 18,
    borderRadius: 9,
    alignItems: 'center',
    justifyContent: 'center',
    paddingHorizontal: 4,
    borderWidth: 2,
    borderColor: '#000000',
  },
  badgeText: {
    fontSize: 10,
    fontWeight: '600',
    color: '#FFFFFF',
  },
  avatar: {
    width: 32,
    height: 32,
    borderRadius: 16,
    backgroundColor: '#171717',
    alignItems: 'center',
    justifyContent: 'center',
    borderWidth: 1,
    borderColor: '#262626',
    marginTop: 4,
  },
  avatarActive: {
    borderColor: '#00D4FF',
  },
});
