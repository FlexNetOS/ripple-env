import React from 'react';
import { View, StyleSheet } from 'react-native';
import { IconNavigation } from './icon-navigation';
import { DetailSidebar } from './detail-sidebar';
import { NotificationsPanel } from './notifications-panel';
import { FavoritesPanel } from './favorites-panel';
import { SidebarProvider } from '@/contexts/sidebar-context';

interface TwoLevelSidebarProps {
  children?: React.ReactNode;
}

export function TwoLevelSidebar({ children }: TwoLevelSidebarProps) {
  return (
    <SidebarProvider>
      <View style={styles.container}>
        <View style={styles.sidebar}>
          <IconNavigation />
          <DetailSidebar />
        </View>
        {children && (
          <View style={styles.content}>
            {children}
          </View>
        )}
        {/* Modal Panels */}
        <NotificationsPanel />
        <FavoritesPanel />
      </View>
    </SidebarProvider>
  );
}

// Standalone sidebar for drawer usage
export function SidebarContent() {
  return (
    <View style={styles.sidebarOnly}>
      <IconNavigation />
      <DetailSidebar />
      {/* Modal Panels */}
      <NotificationsPanel />
      <FavoritesPanel />
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    flexDirection: 'row',
    backgroundColor: '#0A0A0A',
  },
  sidebar: {
    flexDirection: 'row',
    height: '100%',
  },
  sidebarOnly: {
    flexDirection: 'row',
    flex: 1,
    backgroundColor: '#000000',
  },
  content: {
    flex: 1,
  },
});
