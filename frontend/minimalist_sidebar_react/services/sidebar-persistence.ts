/**
 * Sidebar Persistence Service
 * Saves and restores sidebar state using AsyncStorage
 */

import AsyncStorage from '@react-native-async-storage/async-storage';
import { WorkspaceType } from '@/contexts/sidebar-context';

const STORAGE_KEYS = {
  SIDEBAR_STATE: '@ripple/sidebar_state',
  EXPANDED_ITEMS: '@ripple/expanded_items',
  ACTIVE_SECTION: '@ripple/active_section',
  ACTIVE_WORKSPACE: '@ripple/active_workspace',
  DETAIL_COLLAPSED: '@ripple/detail_collapsed',
};

export interface SidebarPersistenceState {
  isOpen: boolean;
  activeSection: string;
  activeWorkspace: WorkspaceType;
  expandedItems: string[];
  detailCollapsed: boolean;
}

const DEFAULT_STATE: SidebarPersistenceState = {
  isOpen: true,
  activeSection: 'ai',
  activeWorkspace: 'all',
  expandedItems: [],
  detailCollapsed: false,
};

class SidebarPersistenceService {
  private cache: SidebarPersistenceState | null = null;

  /**
   * Save the complete sidebar state
   */
  async saveState(state: Partial<SidebarPersistenceState>): Promise<void> {
    try {
      const currentState = await this.loadState();
      const newState = { ...currentState, ...state };
      await AsyncStorage.setItem(STORAGE_KEYS.SIDEBAR_STATE, JSON.stringify(newState));
      this.cache = newState;
    } catch (error) {
      console.error('Failed to save sidebar state:', error);
    }
  }

  /**
   * Load the complete sidebar state
   */
  async loadState(): Promise<SidebarPersistenceState> {
    if (this.cache) {
      return this.cache;
    }

    try {
      const stored = await AsyncStorage.getItem(STORAGE_KEYS.SIDEBAR_STATE);
      if (stored) {
        const parsed = JSON.parse(stored);
        const loadedState = { ...DEFAULT_STATE, ...parsed };
        this.cache = loadedState;
        return loadedState;
      }
    } catch (error) {
      console.error('Failed to load sidebar state:', error);
    }

    return DEFAULT_STATE;
  }

  /**
   * Save expanded items
   */
  async saveExpandedItems(items: string[]): Promise<void> {
    try {
      await AsyncStorage.setItem(STORAGE_KEYS.EXPANDED_ITEMS, JSON.stringify(items));
      if (this.cache) {
        this.cache.expandedItems = items;
      }
    } catch (error) {
      console.error('Failed to save expanded items:', error);
    }
  }

  /**
   * Load expanded items
   */
  async loadExpandedItems(): Promise<string[]> {
    try {
      const stored = await AsyncStorage.getItem(STORAGE_KEYS.EXPANDED_ITEMS);
      if (stored) {
        return JSON.parse(stored);
      }
    } catch (error) {
      console.error('Failed to load expanded items:', error);
    }
    return [];
  }

  /**
   * Save active section
   */
  async saveActiveSection(section: string): Promise<void> {
    try {
      await AsyncStorage.setItem(STORAGE_KEYS.ACTIVE_SECTION, section);
      if (this.cache) {
        this.cache.activeSection = section;
      }
    } catch (error) {
      console.error('Failed to save active section:', error);
    }
  }

  /**
   * Load active section
   */
  async loadActiveSection(): Promise<string> {
    try {
      const stored = await AsyncStorage.getItem(STORAGE_KEYS.ACTIVE_SECTION);
      if (stored) {
        return stored;
      }
    } catch (error) {
      console.error('Failed to load active section:', error);
    }
    return 'ai';
  }

  /**
   * Save active workspace
   */
  async saveActiveWorkspace(workspace: WorkspaceType): Promise<void> {
    try {
      await AsyncStorage.setItem(STORAGE_KEYS.ACTIVE_WORKSPACE, workspace);
      if (this.cache) {
        this.cache.activeWorkspace = workspace;
      }
    } catch (error) {
      console.error('Failed to save active workspace:', error);
    }
  }

  /**
   * Load active workspace
   */
  async loadActiveWorkspace(): Promise<WorkspaceType> {
    try {
      const stored = await AsyncStorage.getItem(STORAGE_KEYS.ACTIVE_WORKSPACE);
      if (stored && ['all', 'devops', 'agents', 'infrastructure', 'settings'].includes(stored)) {
        return stored as WorkspaceType;
      }
    } catch (error) {
      console.error('Failed to load active workspace:', error);
    }
    return 'all';
  }

  /**
   * Save detail sidebar collapsed state
   */
  async saveDetailCollapsed(collapsed: boolean): Promise<void> {
    try {
      await AsyncStorage.setItem(STORAGE_KEYS.DETAIL_COLLAPSED, JSON.stringify(collapsed));
      if (this.cache) {
        this.cache.detailCollapsed = collapsed;
      }
    } catch (error) {
      console.error('Failed to save detail collapsed state:', error);
    }
  }

  /**
   * Load detail sidebar collapsed state
   */
  async loadDetailCollapsed(): Promise<boolean> {
    try {
      const stored = await AsyncStorage.getItem(STORAGE_KEYS.DETAIL_COLLAPSED);
      if (stored) {
        return JSON.parse(stored);
      }
    } catch (error) {
      console.error('Failed to load detail collapsed state:', error);
    }
    return false;
  }

  /**
   * Clear all sidebar state
   */
  async clearState(): Promise<void> {
    try {
      await AsyncStorage.multiRemove(Object.values(STORAGE_KEYS));
      this.cache = null;
    } catch (error) {
      console.error('Failed to clear sidebar state:', error);
    }
  }
}

// Singleton instance
export const sidebarPersistence = new SidebarPersistenceService();

// React hook for sidebar persistence
import { useState, useEffect, useCallback } from 'react';

export function useSidebarPersistence() {
  const [isLoaded, setIsLoaded] = useState(false);
  const [state, setState] = useState<SidebarPersistenceState>(DEFAULT_STATE);

  // Load state on mount
  useEffect(() => {
    const loadState = async () => {
      const loaded = await sidebarPersistence.loadState();
      setState(loaded);
      setIsLoaded(true);
    };
    loadState();
  }, []);

  // Save expanded items
  const saveExpandedItems = useCallback(async (items: Set<string>) => {
    const itemsArray = Array.from(items);
    await sidebarPersistence.saveExpandedItems(itemsArray);
    setState(prev => ({ ...prev, expandedItems: itemsArray }));
  }, []);

  // Save active section
  const saveActiveSection = useCallback(async (section: string) => {
    await sidebarPersistence.saveActiveSection(section);
    setState(prev => ({ ...prev, activeSection: section }));
  }, []);

  // Save active workspace
  const saveActiveWorkspace = useCallback(async (workspace: WorkspaceType) => {
    await sidebarPersistence.saveActiveWorkspace(workspace);
    setState(prev => ({ ...prev, activeWorkspace: workspace }));
  }, []);

  // Save detail collapsed state
  const saveDetailCollapsed = useCallback(async (collapsed: boolean) => {
    await sidebarPersistence.saveDetailCollapsed(collapsed);
    setState(prev => ({ ...prev, detailCollapsed: collapsed }));
  }, []);

  return {
    isLoaded,
    state,
    saveExpandedItems,
    saveActiveSection,
    saveActiveWorkspace,
    saveDetailCollapsed,
  };
}
