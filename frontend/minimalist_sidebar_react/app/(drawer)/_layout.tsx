import React, { useEffect, useCallback } from 'react';
import { View, StyleSheet, Platform, useWindowDimensions } from 'react-native';
import { Drawer } from 'expo-router/drawer';
import { useRouter, usePathname } from 'expo-router';
import { SidebarProvider, useSidebar } from '@/contexts/sidebar-context';
import { SidebarContent } from '@/components/sidebar';
import { useColors } from '@/hooks/use-colors';

const DRAWER_WIDTH = 344; // 64 (icon rail) + 280 (detail sidebar)

function CustomDrawerContent() {
  return (
    <View style={styles.drawerContent}>
      <SidebarContent />
    </View>
  );
}

// Component to wire up navigation - must be inside Drawer context
function NavigationWiring({ children }: { children: React.ReactNode }) {
  const router = useRouter();
  const pathname = usePathname();
  const { setOnNavigate } = useSidebar();

  // Create a stable navigation callback
  const handleNavigate = useCallback((route: string) => {
    try {
      // Only navigate if we're not already on that route
      if (pathname !== route) {
        router.push(route as any);
      }
    } catch (error) {
      console.warn('Navigation error:', error);
    }
  }, [router, pathname]);

  useEffect(() => {
    // Set up navigation callback only when router is available
    if (router) {
      setOnNavigate(handleNavigate);
    }

    return () => {
      setOnNavigate(undefined);
    };
  }, [handleNavigate, setOnNavigate, router]);

  return <>{children}</>;
}

// Inner layout that uses the sidebar context
function DrawerLayoutInner() {
  const colors = useColors();
  const { width } = useWindowDimensions();
  
  // Use permanent drawer on web for larger screens, slide-over for mobile
  const isLargeScreen = width >= 768;
  const drawerType = Platform.OS === 'web' && isLargeScreen ? 'permanent' : 'front';
  const actualDrawerWidth = Platform.OS === 'web' && isLargeScreen 
    ? DRAWER_WIDTH 
    : Math.min(DRAWER_WIDTH, width * 0.85);

  return (
    <Drawer
      screenOptions={{
        headerShown: false,
        drawerType: drawerType,
        drawerStyle: {
          width: actualDrawerWidth,
          backgroundColor: '#000000',
          borderRightWidth: 1,
          borderRightColor: colors.border,
        },
        overlayColor: 'rgba(0, 0, 0, 0.7)',
        swipeEnabled: Platform.OS !== 'web',
        swipeEdgeWidth: 50,
      }}
      drawerContent={() => <CustomDrawerContent />}
    >
      <Drawer.Screen
        name="index"
        options={{
          title: 'Dashboard',
        }}
      />
      <Drawer.Screen
        name="build"
        options={{
          title: 'Build Pipeline',
        }}
      />
      <Drawer.Screen
        name="agents"
        options={{
          title: 'Agents',
        }}
      />
      <Drawer.Screen
        name="infra"
        options={{
          title: 'Infrastructure',
        }}
      />
      <Drawer.Screen
        name="settings"
        options={{
          title: 'Settings',
        }}
      />
    </Drawer>
  );
}

export default function DrawerLayout() {
  return (
    <SidebarProvider>
      <NavigationWiring>
        <DrawerLayoutInner />
      </NavigationWiring>
    </SidebarProvider>
  );
}

const styles = StyleSheet.create({
  drawerContent: {
    flex: 1,
    backgroundColor: '#000000',
  },
});
