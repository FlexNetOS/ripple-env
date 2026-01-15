/**
 * Vue 3 Composable for Vibe Kanban integration
 * Synced from minimalist_sidebar_react/hooks/use-vibe-kanban.ts
 */
import { ref, computed, onMounted, onUnmounted } from 'vue';
import {
  getVibeKanbanClient,
  type Project,
  type VibeKanbanConfig,
} from '@/services/vibe-kanban';

export function useVibeKanban() {
  const client = getVibeKanbanClient();
  const isConnected = ref(false);
  const isLoading = ref(true);
  const projects = ref<Project[]>([]);
  const error = ref<string | null>(null);
  const config = computed<VibeKanbanConfig>(() => client.getConfig());

  const checkConnection = async () => {
    const available = await client.isAvailable();
    isConnected.value = available;
    return available;
  };

  const loadProjects = async () => {
    const result = await client.getProjects();
    if (result.success && result.data) {
      projects.value = result.data;
      error.value = null;
    } else {
      error.value = result.error || 'Failed to load projects';
    }
  };

  const refresh = async () => {
    isLoading.value = true;
    const connected = await checkConnection();
    if (connected) {
      await loadProjects();
    }
    isLoading.value = false;
  };

  const openDashboard = () => {
    const url = config.value.frontendUrl;
    if (typeof window !== 'undefined') {
      window.open(url, '_blank');
    }
  };

  onMounted(() => {
    refresh();
  });

  return {
    isConnected,
    isLoading,
    projects,
    error,
    config,
    refresh,
    openDashboard,
  };
}

export default useVibeKanban;
