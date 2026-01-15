/**
 * Vue 3 Composable for Sidebar state management
 * Synced from minimalist_sidebar_react/contexts/sidebar-context.tsx
 */
import { ref, computed, watch } from 'vue';
import { navItems, getSidebarContent, type SidebarContent } from '@/data/sidebar-data';

export interface SidebarState {
  activeNavItem: string;
  isExpanded: boolean;
  expandedItems: Set<string>;
}

const state = ref<SidebarState>({
  activeNavItem: 'ai',
  isExpanded: true,
  expandedItems: new Set<string>(),
});

export function useSidebar() {
  const activeContent = computed<SidebarContent>(() => 
    getSidebarContent(state.value.activeNavItem)
  );

  const setActiveNavItem = (itemId: string) => {
    state.value.activeNavItem = itemId;
  };

  const toggleExpanded = () => {
    state.value.isExpanded = !state.value.isExpanded;
  };

  const toggleItemExpanded = (itemId: string) => {
    const newSet = new Set(state.value.expandedItems);
    if (newSet.has(itemId)) {
      newSet.delete(itemId);
    } else {
      newSet.add(itemId);
    }
    state.value.expandedItems = newSet;
  };

  const isItemExpanded = (itemId: string): boolean => {
    return state.value.expandedItems.has(itemId);
  };

  return {
    state,
    navItems,
    activeContent,
    setActiveNavItem,
    toggleExpanded,
    toggleItemExpanded,
    isItemExpanded,
  };
}

export default useSidebar;
