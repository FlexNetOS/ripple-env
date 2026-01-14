import React, { createContext, useContext, useState, useCallback, useMemo, useEffect, ReactNode } from 'react';
import { sidebarPersistence } from '@/services/sidebar-persistence';

export type WorkspaceType = 'all' | 'productivity' | 'communication' | 'intelligence' | 'infrastructure' | 'life' | 'business';

export interface NotificationBadge {
  count?: number;
  status?: 'success' | 'warning' | 'error' | 'info';
  pulse?: boolean;
}

// Route mapping from section IDs to screen routes
export const sectionRouteMap: Record<string, string> = {
  'ai': '/(drawer)',
  'dashboard': '/(drawer)',
  'build': '/(drawer)/build',
  'agents': '/(drawer)/agents',
  'infra': '/(drawer)/infra',
  'settings': '/(drawer)/settings',
  // Additional sections that map to main screens
  'nodes': '/(drawer)/infra',
  'storage': '/(drawer)/infra',
  'logs': '/(drawer)/build',
  'analytics': '/(drawer)',
};

export interface SidebarContextProps {
  state: 'expanded' | 'collapsed';
  isOpen: boolean;
  setOpen: (open: boolean) => void;
  activeSection: string;
  setActiveSection: (section: string) => void;
  activeWorkspace: WorkspaceType;
  setActiveWorkspace: (workspace: WorkspaceType) => void;
  toggleSidebar: () => void;
  expandedItems: Set<string>;
  toggleExpanded: (itemKey: string) => void;
  // Notifications panel
  showNotifications: boolean;
  setShowNotifications: (show: boolean) => void;
  // Favorites panel
  showFavorites: boolean;
  setShowFavorites: (show: boolean) => void;
  // Quick switcher
  showQuickSwitcher: boolean;
  setShowQuickSwitcher: (show: boolean) => void;
  // Navigation callback
  onNavigate?: (route: string) => void;
  setOnNavigate: (callback: ((route: string) => void) | undefined) => void;
  // Detail sidebar collapsed state
  detailCollapsed: boolean;
  setDetailCollapsed: (collapsed: boolean) => void;
  // Persistence loaded flag
  isPersistenceLoaded: boolean;
}

const SidebarContext = createContext<SidebarContextProps | null>(null);

export function useSidebar() {
  const context = useContext(SidebarContext);
  if (!context) {
    throw new Error('useSidebar must be used within a SidebarProvider');
  }
  return context;
}

interface SidebarProviderProps {
  children: ReactNode;
  defaultOpen?: boolean;
}

export function SidebarProvider({ children, defaultOpen = true }: SidebarProviderProps) {
  const [isOpen, setIsOpen] = useState(defaultOpen);
  const [activeSection, setActiveSectionState] = useState('ai');
  const [activeWorkspace, setActiveWorkspaceState] = useState<WorkspaceType>('all');
  const [expandedItems, setExpandedItems] = useState<Set<string>>(new Set());
  const [showNotifications, setShowNotifications] = useState(false);
  const [showFavorites, setShowFavorites] = useState(false);
  const [showQuickSwitcher, setShowQuickSwitcher] = useState(false);
  const [onNavigate, setOnNavigate] = useState<((route: string) => void) | undefined>(undefined);
  const [detailCollapsed, setDetailCollapsedState] = useState(false);
  const [isPersistenceLoaded, setIsPersistenceLoaded] = useState(false);

  // Load persisted state on mount
  useEffect(() => {
    const loadPersistedState = async () => {
      try {
        const state = await sidebarPersistence.loadState();
        setActiveSectionState(state.activeSection);
        setActiveWorkspaceState(state.activeWorkspace);
        setExpandedItems(new Set(state.expandedItems));
        setDetailCollapsedState(state.detailCollapsed);
        setIsPersistenceLoaded(true);
      } catch (error) {
        console.error('Failed to load persisted sidebar state:', error);
        setIsPersistenceLoaded(true);
      }
    };
    loadPersistedState();
  }, []);

  const toggleSidebar = useCallback(() => {
    setIsOpen(prev => !prev);
  }, []);

  const toggleExpanded = useCallback((itemKey: string) => {
    setExpandedItems(prev => {
      const newSet = new Set(prev);
      if (newSet.has(itemKey)) {
        newSet.delete(itemKey);
      } else {
        newSet.add(itemKey);
      }
      // Persist expanded items
      sidebarPersistence.saveExpandedItems(Array.from(newSet));
      return newSet;
    });
  }, []);

  // Enhanced setActiveSection that also triggers navigation and persists
  const setActiveSection = useCallback((section: string) => {
    setActiveSectionState(section);
    sidebarPersistence.saveActiveSection(section);
    
    // Trigger navigation if callback is set
    const route = sectionRouteMap[section];
    if (route && onNavigate) {
      onNavigate(route);
    }
  }, [onNavigate]);

  // Enhanced setActiveWorkspace with persistence
  const setActiveWorkspace = useCallback((workspace: WorkspaceType) => {
    setActiveWorkspaceState(workspace);
    sidebarPersistence.saveActiveWorkspace(workspace);
  }, []);

  // Enhanced setDetailCollapsed with persistence
  const setDetailCollapsed = useCallback((collapsed: boolean) => {
    setDetailCollapsedState(collapsed);
    sidebarPersistence.saveDetailCollapsed(collapsed);
  }, []);

  const state = isOpen ? 'expanded' : 'collapsed';

  const contextValue = useMemo<SidebarContextProps>(() => ({
    state,
    isOpen,
    setOpen: setIsOpen,
    activeSection,
    setActiveSection,
    activeWorkspace,
    setActiveWorkspace,
    toggleSidebar,
    expandedItems,
    toggleExpanded,
    showNotifications,
    setShowNotifications,
    showFavorites,
    setShowFavorites,
    showQuickSwitcher,
    setShowQuickSwitcher,
    onNavigate,
    setOnNavigate,
    detailCollapsed,
    setDetailCollapsed,
    isPersistenceLoaded,
  }), [
    state, 
    isOpen, 
    activeSection, 
    setActiveSection,
    activeWorkspace,
    setActiveWorkspace,
    toggleSidebar, 
    expandedItems, 
    toggleExpanded,
    showNotifications,
    showFavorites,
    showQuickSwitcher,
    onNavigate,
    detailCollapsed,
    setDetailCollapsed,
    isPersistenceLoaded,
  ]);

  return (
    <SidebarContext.Provider value={contextValue}>
      {children}
    </SidebarContext.Provider>
  );
}
