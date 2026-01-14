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
  // Double minimize support - synced with web
  iconCollapsed: boolean;
  setIconCollapsed: (collapsed: boolean) => void;
  // Detail sidebar collapsed state
  detailCollapsed: boolean;
  setDetailCollapsed: (collapsed: boolean) => void;
  // Double minimize toggle
  toggleDoubleMinimize: () => void;
  isFullyCollapsed: boolean;
  // Notifications panel
  showNotifications: boolean;
  setShowNotifications: (show: boolean) => void;
  // Favorites panel
  showFavorites: boolean;
  setShowFavorites: (show: boolean) => void;
  // Quick switcher
  showQuickSwitcher: boolean;
  setShowQuickSwitcher: (show: boolean) => void;
  // Web browser panel - synced with web
  showWebBrowser: boolean;
  setShowWebBrowser: (show: boolean) => void;
  browserUrl: string;
  setBrowserUrl: (url: string) => void;
  // Navigation callback
  onNavigate?: (route: string) => void;
  setOnNavigate: (callback: ((route: string) => void) | undefined) => void;
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
  const [iconCollapsed, setIconCollapsedState] = useState(false);
  const [showNotifications, setShowNotifications] = useState(false);
  const [showFavorites, setShowFavorites] = useState(false);
  const [showQuickSwitcher, setShowQuickSwitcher] = useState(false);
  const [showWebBrowser, setShowWebBrowserState] = useState(false);
  const [browserUrl, setBrowserUrlState] = useState('https://google.com');
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
        setIconCollapsedState(state.iconCollapsed || false);
        setBrowserUrlState(state.browserUrl || 'https://google.com');
        setIsPersistenceLoaded(true);
      } catch (error) {
        console.error('Failed to load persisted sidebar state:', error);
        setIsPersistenceLoaded(true);
      }
    };
    loadPersistedState();
  }, []);

  // Icon collapsed setter with persistence
  const setIconCollapsed = useCallback((collapsed: boolean) => {
    setIconCollapsedState(collapsed);
    sidebarPersistence.saveIconCollapsed(collapsed);
  }, []);

  // Double minimize toggle
  const toggleDoubleMinimize = useCallback(() => {
    const isCurrentlyFullyCollapsed = iconCollapsed && detailCollapsed;
    if (isCurrentlyFullyCollapsed) {
      setIconCollapsedState(false);
      setDetailCollapsedState(false);
      sidebarPersistence.saveIconCollapsed(false);
      sidebarPersistence.saveDetailCollapsed(false);
    } else {
      setIconCollapsedState(true);
      setDetailCollapsedState(true);
      sidebarPersistence.saveIconCollapsed(true);
      sidebarPersistence.saveDetailCollapsed(true);
    }
  }, [iconCollapsed, detailCollapsed]);

  const isFullyCollapsed = iconCollapsed && detailCollapsed;

  // Web browser controls
  const setShowWebBrowser = useCallback((show: boolean) => {
    setShowWebBrowserState(show);
    if (show) {
      setShowNotifications(false);
      setShowFavorites(false);
      setShowQuickSwitcher(false);
    }
  }, []);

  const setBrowserUrl = useCallback((url: string) => {
    setBrowserUrlState(url);
    sidebarPersistence.saveBrowserUrl(url);
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
    iconCollapsed,
    setIconCollapsed,
    detailCollapsed,
    setDetailCollapsed,
    toggleDoubleMinimize,
    isFullyCollapsed,
    showNotifications,
    setShowNotifications,
    showFavorites,
    setShowFavorites,
    showQuickSwitcher,
    setShowQuickSwitcher,
    showWebBrowser,
    setShowWebBrowser,
    browserUrl,
    setBrowserUrl,
    onNavigate,
    setOnNavigate,
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
    iconCollapsed,
    setIconCollapsed,
    detailCollapsed,
    setDetailCollapsed,
    toggleDoubleMinimize,
    isFullyCollapsed,
    showNotifications,
    showFavorites,
    showQuickSwitcher,
    showWebBrowser,
    setShowWebBrowser,
    browserUrl,
    setBrowserUrl,
    onNavigate,
    isPersistenceLoaded,
  ]);

  return (
    <SidebarContext.Provider value={contextValue}>
      {children}
    </SidebarContext.Provider>
  );
}
