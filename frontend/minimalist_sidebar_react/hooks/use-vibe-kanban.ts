/**
 * React hook for Vibe Kanban integration
 */
import { useEffect, useState, useCallback } from 'react';
import { 
  getVibeKanbanClient, 
  type Project, 
  type VibeKanbanConfig 
} from '@/services/vibe-kanban';

export interface UseVibeKanbanResult {
  isConnected: boolean;
  isLoading: boolean;
  projects: Project[];
  error: string | null;
  config: VibeKanbanConfig;
  refresh: () => Promise<void>;
  openDashboard: () => void;
}

export function useVibeKanban(): UseVibeKanbanResult {
  const client = getVibeKanbanClient();
  const [isConnected, setIsConnected] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [projects, setProjects] = useState<Project[]>([]);
  const [error, setError] = useState<string | null>(null);
  const config = client.getConfig();

  const checkConnection = useCallback(async () => {
    const available = await client.isAvailable();
    setIsConnected(available);
    return available;
  }, [client]);

  const loadProjects = useCallback(async () => {
    const result = await client.getProjects();
    if (result.success && result.data) {
      setProjects(result.data);
      setError(null);
    } else {
      setError(result.error || 'Failed to load projects');
    }
  }, [client]);

  const refresh = useCallback(async () => {
    setIsLoading(true);
    const connected = await checkConnection();
    if (connected) {
      await loadProjects();
    }
    setIsLoading(false);
  }, [checkConnection, loadProjects]);

  const openDashboard = useCallback(() => {
    const url = config.frontendUrl;
    if (typeof window !== 'undefined') {
      window.open(url, '_blank');
    }
  }, [config.frontendUrl]);

  useEffect(() => {
    refresh();
  }, [refresh]);

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
