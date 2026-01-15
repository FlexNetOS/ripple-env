import React, { useState } from 'react';
import { View, Text, Pressable, TextInput, ScrollView, StyleSheet } from 'react-native';
import { useSidebar } from '@/contexts/sidebar-context';
import { ALL_PANELS, MenuItem as PanelMenuItem, MenuSection as PanelMenuSection } from '@/data/sidebar-panels';
import { useColors } from '@/hooks/use-colors';
import Animated, { useAnimatedStyle, withTiming, withSpring } from 'react-native-reanimated';
import MaterialIcons from '@expo/vector-icons/MaterialIcons';
import { useSafeAreaInsets } from 'react-native-safe-area-context';

// Icon mapping from panel icons to MaterialIcons
const iconMap: Record<string, keyof typeof MaterialIcons.glyphMap> = {
  'message-square': 'chat',
  'target': 'gps-fixed',
  'lightbulb': 'lightbulb',
  'shield': 'shield',
  'brain': 'psychology',
  'sparkles': 'auto-awesome',
  'robot': 'smart-toy',
  'bot': 'smart-toy',
  'workflow': 'account-tree',
  'layout-grid': 'grid-view',
  'play': 'play-arrow',
  'check-circle': 'check-circle',
  'loader': 'sync',
  'clock': 'schedule',
  'server': 'dns',
  'cpu': 'memory',
  'hammer': 'build',
  'users': 'groups',
  'git-branch': 'call-split',
  'trending-up': 'trending-up',
  'link': 'link',
  'database': 'storage',
  'arrow-down': 'arrow-downward',
  'arrow-up': 'arrow-upward',
  'palette': 'palette',
  'bell': 'notifications',
  'globe': 'language',
  'terminal': 'terminal',
  'key': 'vpn-key',
  'plug': 'power',
  'info': 'info',
  'book': 'menu-book',
  'help-circle': 'help',
  'file-text': 'description',
  'rotate-ccw': 'replay',
  'check': 'check',
  'calendar': 'event',
  'calendar-check': 'event-available',
  'moon': 'dark-mode',
  'zap': 'bolt',
  'rocket': 'rocket-launch',
  'plane': 'flight',
  'heart': 'favorite',
  'dollar': 'attach-money',
  'balance': 'balance',
  'shield-check': 'verified-user',
  'lock': 'lock',
  'sliders': 'tune',
  'activity': 'show-chart',
  'mic': 'mic',
  'eye': 'visibility',
  'type': 'text-fields',
  'briefcase': 'work',
  'home': 'home',
  'sun': 'wb-sunny',
  'scale': 'scale',
  'minus': 'remove',
  'plus': 'add',
  'video': 'videocam',
  'phone': 'phone',
  'message-circle': 'chat-bubble',
  'map-pin': 'location-on',
  'landmark': 'account-balance',
  'credit-card': 'credit-card',
  'repeat': 'repeat',
  'alert-triangle': 'warning',
  'pie-chart': 'pie-chart',
  'bar-chart': 'bar-chart',
  'bar-chart-2': 'bar-chart',
  'calculator': 'calculate',
  'check-square': 'check-box',
  'clipboard-list': 'assignment',
  'pen-tool': 'edit',
  'folder': 'folder',
  'folder-open': 'folder-open',
  'kanban': 'view-kanban',
  'list': 'list',
  'archive': 'archive',
  'file': 'insert-drive-file',
  'image': 'image',
  'download': 'download',
  'upload': 'upload',
  'share': 'share',
  'share-2': 'share',
  'instagram': 'photo-camera',
  'twitter': 'chat',
  'linkedin': 'work',
  'youtube': 'play-circle',
  'rss': 'rss-feed',
  'monitor': 'monitor',
  'hard-drive': 'storage',
  'wifi': 'wifi',
  'cloud': 'cloud',
  'mail': 'mail',
  'inbox': 'inbox',
  'send': 'send',
  'star': 'star',
  'user': 'person',
  'settings': 'settings',
  'log-out': 'logout',
};

// Get panel content by ID
function getPanelContent(panelId: string) {
  const panel = ALL_PANELS.find(p => p.id === panelId);
  if (panel) {
    return {
      title: panel.title,
      badge: panel.badge,
      badgeColor: panel.badgeColor,
      sections: panel.sections,
    };
  }
  // Default to first panel if not found
  return {
    title: ALL_PANELS[0].title,
    badge: ALL_PANELS[0].badge,
    badgeColor: ALL_PANELS[0].badgeColor,
    sections: ALL_PANELS[0].sections,
  };
}

interface BadgeProps {
  count?: number;
  color?: string;
  small?: boolean;
}

function Badge({ count, color = '#EF4444', small }: BadgeProps) {
  if (count === undefined) return null;

  return (
    <View style={[styles.badge, small && styles.badgeSmall, { backgroundColor: color }]}>
      <Text style={[styles.badgeText, small && styles.badgeTextSmall]}>
        {count > 99 ? '99+' : count}
      </Text>
    </View>
  );
}

interface StatusDotProps {
  status?: string;
}

function StatusDot({ status }: StatusDotProps) {
  if (!status) return null;

  const colors: Record<string, string> = {
    online: '#00E676',
    offline: '#666666',
    away: '#FFB300',
    busy: '#FF5252',
  };

  return (
    <View style={[styles.statusDot, { backgroundColor: colors[status] || '#666666' }]} />
  );
}

interface MenuItemProps {
  item: PanelMenuItem;
  onPress?: (itemId: string) => void;
}

function MenuItem({ item, onPress }: MenuItemProps) {
  const colors = useColors();
  const iconName = iconMap[item.icon] || 'circle';

  return (
    <Pressable
      style={({ pressed }) => [
        styles.menuItem,
        pressed && styles.menuItemPressed,
      ]}
      onPress={() => onPress?.(item.id)}
    >
      <View style={styles.menuItemLeft}>
        <View style={styles.iconContainer}>
          <MaterialIcons name={iconName} size={16} color={colors.foreground} />
          {item.status && <StatusDot status={item.status} />}
        </View>
        <View style={styles.menuItemContent}>
          <Text style={[styles.menuItemLabel, { color: colors.foreground }]} numberOfLines={1}>
            {item.label}
          </Text>
          {item.metadata && (
            <Text style={[styles.menuItemMetadata, { color: colors.muted }]} numberOfLines={1}>
              {item.metadata}
            </Text>
          )}
        </View>
      </View>
      <View style={styles.menuItemRight}>
        {item.badge !== undefined && <Badge count={item.badge} color={item.badgeColor} small />}
        <MaterialIcons name="chevron-right" size={16} color={colors.muted} />
      </View>
    </Pressable>
  );
}

interface MenuSectionProps {
  section: PanelMenuSection;
  onItemPress?: (itemId: string) => void;
}

function MenuSection({ section, onItemPress }: MenuSectionProps) {
  const colors = useColors();
  const [isExpanded, setIsExpanded] = useState(section.defaultExpanded !== false);

  const animatedChevron = useAnimatedStyle(() => ({
    transform: [{ rotate: withTiming(isExpanded ? '0deg' : '-90deg', { duration: 200 }) }],
  }));

  return (
    <View style={styles.section}>
      <Pressable 
        style={styles.sectionHeader}
        onPress={() => section.collapsible && setIsExpanded(!isExpanded)}
      >
        <Text style={[styles.sectionTitle, { color: colors.muted }]}>
          {section.title}
        </Text>
        {section.collapsible && (
          <Animated.View style={animatedChevron}>
            <MaterialIcons name="expand-more" size={16} color={colors.muted} />
          </Animated.View>
        )}
      </Pressable>
      {isExpanded && (
        <View style={styles.sectionItems}>
          {section.items.map((item) => (
            <MenuItem key={item.id} item={item} onPress={onItemPress} />
          ))}
        </View>
      )}
    </View>
  );
}

function SearchBar() {
  const colors = useColors();
  const [searchValue, setSearchValue] = useState('');

  return (
    <View style={[styles.searchContainer, { borderColor: colors.border }]}>
      <MaterialIcons name="search" size={16} color={colors.muted} />
      <TextInput
        style={[styles.searchInput, { color: colors.foreground }]}
        placeholder="Search or press ⌘K..."
        placeholderTextColor={colors.muted}
        value={searchValue}
        onChangeText={setSearchValue}
      />
      <View style={[styles.shortcutHint, { backgroundColor: colors.surface }]}>
        <Text style={[styles.shortcutText, { color: colors.muted }]}>⌘K</Text>
      </View>
    </View>
  );
}

interface HeaderProps {
  title: string;
  badge?: number;
  badgeColor?: string;
  onToggleCollapse: () => void;
  isCollapsed: boolean;
}

function Header({ title, badge, badgeColor, onToggleCollapse, isCollapsed }: HeaderProps) {
  const colors = useColors();

  const animatedChevron = useAnimatedStyle(() => ({
    transform: [{ rotate: withTiming(isCollapsed ? '180deg' : '0deg', { duration: 200 }) }],
  }));

  return (
    <View style={styles.header}>
      <View style={styles.headerLeft}>
        <Text style={[styles.headerTitle, { color: colors.foreground }]}>{title}</Text>
        {badge !== undefined && <Badge count={badge} color={badgeColor} />}
      </View>
      <Pressable onPress={onToggleCollapse} style={styles.collapseButton}>
        <Animated.View style={animatedChevron}>
          <MaterialIcons name="chevron-left" size={20} color={colors.muted} />
        </Animated.View>
      </Pressable>
    </View>
  );
}

export function DetailSidebar() {
  const { activeSection, onNavigate } = useSidebar();
  const colors = useColors();
  const insets = useSafeAreaInsets();
  const [isCollapsed, setIsCollapsed] = useState(false);
  
  // Get content from sidebar-panels.ts based on activeSection
  const content = getPanelContent(activeSection);

  const animatedWidth = useAnimatedStyle(() => ({
    width: withTiming(isCollapsed ? 64 : 280, { duration: 300 }),
  }));

  // Handle menu item clicks - navigate if there's a route mapping
  const handleItemPress = (itemId: string) => {
    // Import route mapping from context
    const { sectionRouteMap } = require('@/contexts/sidebar-context');
    const route = sectionRouteMap[itemId];
    if (route && onNavigate) {
      onNavigate(route);
    }
  };

  return (
    <Animated.View style={[
      styles.container, 
      { 
        backgroundColor: '#000000',
        paddingTop: insets.top + 8, // Add safe area padding for status bar
      }, 
      animatedWidth
    ]}>
      <Header
        title={content.title}
        badge={content.badge}
        badgeColor={content.badgeColor}
        onToggleCollapse={() => setIsCollapsed(!isCollapsed)}
        isCollapsed={isCollapsed}
      />

      {!isCollapsed && (
        <>
          <SearchBar />
          <ScrollView style={styles.scrollContainer} showsVerticalScrollIndicator={false}>
            {content.sections.map((section) => (
              <MenuSection key={section.id} section={section} onItemPress={handleItemPress} />
            ))}
          </ScrollView>
        </>
      )}
    </Animated.View>
  );
}

const styles = StyleSheet.create({
  container: {
    height: '100%',
    paddingHorizontal: 16,
    paddingBottom: 16,
  },
  header: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: 16,
    height: 40,
  },
  headerLeft: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  headerTitle: {
    fontSize: 18,
    fontWeight: '600',
  },
  collapseButton: {
    width: 32,
    height: 32,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
  },
  searchContainer: {
    flexDirection: 'row',
    alignItems: 'center',
    height: 40,
    borderRadius: 8,
    borderWidth: 1,
    paddingHorizontal: 12,
    marginBottom: 20,
    gap: 8,
  },
  searchInput: {
    flex: 1,
    fontSize: 14,
    height: '100%',
  },
  shortcutHint: {
    paddingHorizontal: 6,
    paddingVertical: 2,
    borderRadius: 4,
  },
  shortcutText: {
    fontSize: 10,
    fontWeight: '500',
  },
  scrollContainer: {
    flex: 1,
  },
  section: {
    marginBottom: 24,
  },
  sectionHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    marginBottom: 8,
  },
  sectionTitle: {
    fontSize: 11,
    fontWeight: '600',
    letterSpacing: 0.5,
  },
  sectionItems: {
    gap: 2,
  },
  menuItem: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    paddingHorizontal: 12,
    paddingVertical: 10,
    borderRadius: 8,
  },
  menuItemPressed: {
    backgroundColor: 'rgba(255, 255, 255, 0.05)',
  },
  menuItemLeft: {
    flexDirection: 'row',
    alignItems: 'center',
    flex: 1,
    gap: 12,
  },
  iconContainer: {
    position: 'relative',
  },
  menuItemContent: {
    flex: 1,
  },
  menuItemLabel: {
    fontSize: 14,
    fontWeight: '500',
  },
  menuItemMetadata: {
    fontSize: 11,
    marginTop: 2,
  },
  menuItemRight: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  badge: {
    minWidth: 20,
    height: 20,
    borderRadius: 10,
    alignItems: 'center',
    justifyContent: 'center',
    paddingHorizontal: 6,
  },
  badgeSmall: {
    minWidth: 16,
    height: 16,
    borderRadius: 8,
    paddingHorizontal: 4,
  },
  badgeText: {
    fontSize: 11,
    fontWeight: '600',
    color: '#FFFFFF',
  },
  badgeTextSmall: {
    fontSize: 9,
  },
  statusDot: {
    position: 'absolute',
    bottom: -2,
    right: -2,
    width: 6,
    height: 6,
    borderRadius: 3,
    borderWidth: 1,
    borderColor: '#000000',
  },
});
