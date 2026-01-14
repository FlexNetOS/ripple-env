import React, { createContext, useContext, useState, useCallback, useMemo, useEffect, ReactNode } from 'react';
import { WorkspaceType, SECTION_ROUTES } from '@/data/sidebar-panels';

export interface SidebarContextProps {
  activeSection: string;
  setActiveSection: (section: string) => void;
  activeWorkspace: WorkspaceType;
  setActiveWorkspace: (workspace: WorkspaceType) => void;
  expandedItems: Set<string>;
  toggleExpanded: (itemKey: string) => void;
  detailCollapsed: boolean;
  setDetailCollapsed: (collapsed: boolean) => void;
  showNotifications: boolean;
  setShowNotifications: (show: boolean) => void;
  showFavorites: boolean;
  setShowFavorites: (show: boolean) => void;
  showCommandPalette: boolean;
  setShowCommandPalette: (show: boolean) => void;
  onNavigate?: (route: string) => void;
  setOnNavigate: (callback: ((route: string) => void) | undefined) => void;
}

const SidebarContext = createContext<SidebarContextProps | null>(null);

export function useSidebar() {
  const context = useContext(SidebarContext);
  if (!context) {
    throw new Error('useSidebar must be used within a SidebarProvider');
  }
  return context;
}

// Persistence helpers
const STORAGE_KEYS = {
  activeSection: 'sidebar_activeSection',
  activeWorkspace: 'sidebar_activeWorkspace',
  expandedItems: 'sidebar_expandedItems',
  detailCollapsed: 'sidebar_detailCollapsed',
};

function loadFromStorage<T>(key: string, defaultValue: T): T {
  try {
    const stored = localStorage.getItem(key);
    if (stored) {
      return JSON.parse(stored);
    }
  } catch (e) {
    console.warn('Failed to load from storage:', key, e);
  }
  return defaultValue;
}

function saveToStorage<T>(key: string, value: T): void {
  try {
    localStorage.setItem(key, JSON.stringify(value));
  } catch (e) {
    console.warn('Failed to save to storage:', key, e);
  }
}

interface SidebarProviderProps {
  children: ReactNode;
}

export function SidebarProvider({ children }: SidebarProviderProps) {
  const [activeSection, setActiveSectionState] = useState('ai-command-center');
  const [activeWorkspace, setActiveWorkspaceState] = useState<WorkspaceType>('all');
  const [expandedItems, setExpandedItems] = useState<Set<string>>(new Set());
  const [detailCollapsed, setDetailCollapsedState] = useState(false);
  const [showNotifications, setShowNotificationsState] = useState(false);
  const [showFavorites, setShowFavoritesState] = useState(false);
  const [showCommandPalette, setShowCommandPaletteState] = useState(false);
  const [onNavigate, setOnNavigateState] = useState<((route: string) => void) | undefined>(undefined);

  // Load persisted state on mount
  useEffect(() => {
    const savedSection = loadFromStorage(STORAGE_KEYS.activeSection, 'ai-command-center');
    const savedWorkspace = loadFromStorage<WorkspaceType>(STORAGE_KEYS.activeWorkspace, 'all');
    const savedExpanded = loadFromStorage<string[]>(STORAGE_KEYS.expandedItems, []);
    const savedCollapsed = loadFromStorage(STORAGE_KEYS.detailCollapsed, false);

    setActiveSectionState(savedSection);
    setActiveWorkspaceState(savedWorkspace);
    setExpandedItems(new Set(savedExpanded));
    setDetailCollapsedState(savedCollapsed);
  }, []);

  // Keyboard shortcut for command palette (⌘K / Ctrl+K)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if ((e.metaKey || e.ctrlKey) && e.key === 'k') {
        e.preventDefault();
        setShowCommandPaletteState(prev => !prev);
      }
      // Escape to close any open panel
      if (e.key === 'Escape') {
        setShowNotificationsState(false);
        setShowFavoritesState(false);
        setShowCommandPaletteState(false);
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, []);

  const toggleExpanded = useCallback((itemKey: string) => {
    setExpandedItems(prev => {
      const newSet = new Set(prev);
      if (newSet.has(itemKey)) {
        newSet.delete(itemKey);
      } else {
        newSet.add(itemKey);
      }
      saveToStorage(STORAGE_KEYS.expandedItems, Array.from(newSet));
      return newSet;
    });
  }, []);

  const setActiveSection = useCallback((section: string) => {
    setActiveSectionState(section);
    saveToStorage(STORAGE_KEYS.activeSection, section);
    
    // Trigger navigation if callback is set
    const route = SECTION_ROUTES[section];
    if (route && onNavigate) {
      onNavigate(route);
    }
  }, [onNavigate]);

  const setActiveWorkspace = useCallback((workspace: WorkspaceType) => {
    setActiveWorkspaceState(workspace);
    saveToStorage(STORAGE_KEYS.activeWorkspace, workspace);
  }, []);

  const setDetailCollapsed = useCallback((collapsed: boolean) => {
    setDetailCollapsedState(collapsed);
    saveToStorage(STORAGE_KEYS.detailCollapsed, collapsed);
  }, []);

  const setShowNotifications = useCallback((show: boolean) => {
    setShowNotificationsState(show);
    if (show) {
      setShowFavoritesState(false);
      setShowCommandPaletteState(false);
    }
  }, []);

  const setShowFavorites = useCallback((show: boolean) => {
    setShowFavoritesState(show);
    if (show) {
      setShowNotificationsState(false);
      setShowCommandPaletteState(false);
    }
  }, []);

  const setShowCommandPalette = useCallback((show: boolean) => {
    setShowCommandPaletteState(show);
    if (show) {
      setShowNotificationsState(false);
      setShowFavoritesState(false);
    }
  }, []);

  const setOnNavigate = useCallback((callback: ((route: string) => void) | undefined) => {
    setOnNavigateState(() => callback);
  }, []);

  const contextValue = useMemo<SidebarContextProps>(() => ({
    activeSection,
    setActiveSection,
    activeWorkspace,
    setActiveWorkspace,
    expandedItems,
    toggleExpanded,
    detailCollapsed,
    setDetailCollapsed,
    showNotifications,
    setShowNotifications,
    showFavorites,
    setShowFavorites,
    showCommandPalette,
    setShowCommandPalette,
    onNavigate,
    setOnNavigate,
  }), [
    activeSection, 
    setActiveSection,
    activeWorkspace,
    setActiveWorkspace,
    expandedItems, 
    toggleExpanded,
    detailCollapsed,
    setDetailCollapsed,
    showNotifications,
    setShowNotifications,
    showFavorites,
    setShowFavorites,
    showCommandPalette,
    setShowCommandPalette,
    onNavigate,
    setOnNavigate,
  ]);

  return (
    <SidebarContext.Provider value={contextValue}>
      {children}
    </SidebarContext.Provider>
  );
}
