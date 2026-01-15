/**
 * Vibe Kanban Integration Service
 * Connects the minimalist_sidebar (Vue) to the existing vibe-kanban application.
 * Synced from minimalist_sidebar_react
 */

// Configuration for vibe-kanban service
export interface VibeKanbanConfig {
  apiBaseUrl: string;
  wsBaseUrl: string;
  frontendUrl: string;
}

export function getVibeKanbanConfig(): VibeKanbanConfig {
  const host = typeof window !== 'undefined' ? window.location.hostname : '127.0.0.1';
  const defaultBackendPort = import.meta.env.VITE_VIBE_KANBAN_BACKEND_PORT || '3001';
  const defaultFrontendPort = import.meta.env.VITE_VIBE_KANBAN_FRONTEND_PORT || '3000';

  return {
    apiBaseUrl: import.meta.env.VITE_VIBE_KANBAN_API_URL || `http://${host}:${defaultBackendPort}`,
    wsBaseUrl: import.meta.env.VITE_VIBE_KANBAN_WS_URL || `ws://${host}:${defaultBackendPort}`,
    frontendUrl: import.meta.env.VITE_VIBE_KANBAN_FRONTEND_URL || `http://${host}:${defaultFrontendPort}`,
  };
}

// Types from vibe-kanban
export interface Project {
  id: string;
  name: string;
  path: string;
  created_at: string;
  updated_at?: string;
  archived?: boolean;
}

export interface Task {
  id: string;
  project_id: string;
  title: string;
  description?: string;
  status: 'pending' | 'in_progress' | 'completed' | 'archived';
  created_at: string;
  updated_at?: string;
}

export interface ApiResponse<T> {
  success: boolean;
  data?: T;
  error?: string;
}

export class VibeKanbanClient {
  private config: VibeKanbanConfig;
  private wsConnection: WebSocket | null = null;

  constructor(config?: Partial<VibeKanbanConfig>) {
    this.config = { ...getVibeKanbanConfig(), ...config };
  }

  async isAvailable(): Promise<boolean> {
    try {
      const response = await fetch(`${this.config.apiBaseUrl}/api/health`, {
        method: 'GET',
        headers: { 'Accept': 'application/json' },
      });
      return response.ok;
    } catch {
      return false;
    }
  }

  getEmbedUrl(): string {
    return this.config.frontendUrl;
  }

  async getProjects(): Promise<ApiResponse<Project[]>> {
    try {
      const response = await fetch(`${this.config.apiBaseUrl}/api/projects`, {
        method: 'GET',
        headers: { 'Accept': 'application/json' },
      });
      if (!response.ok) {
        return { success: false, error: `HTTP ${response.status}` };
      }
      const data = await response.json();
      return { success: true, data: data.projects || data };
    } catch (error) {
      return { success: false, error: String(error) };
    }
  }

  async getProjectTasks(projectId: string): Promise<ApiResponse<Task[]>> {
    try {
      const response = await fetch(`${this.config.apiBaseUrl}/api/projects/${projectId}/tasks`, {
        method: 'GET',
        headers: { 'Accept': 'application/json' },
      });
      if (!response.ok) {
        return { success: false, error: `HTTP ${response.status}` };
      }
      const data = await response.json();
      return { success: true, data: data.tasks || data };
    } catch (error) {
      return { success: false, error: String(error) };
    }
  }

  connectToStream(
    endpoint: string,
    onMessage: (data: unknown) => void,
    onError?: (error: Event) => void
  ): () => void {
    const wsUrl = `${this.config.wsBaseUrl}${endpoint}`;
    try {
      this.wsConnection = new WebSocket(wsUrl);
      this.wsConnection.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          onMessage(data);
        } catch (e) {
          console.warn('Failed to parse WebSocket message:', e);
        }
      };
      this.wsConnection.onerror = (error) => onError?.(error);
    } catch (error) {
      console.error('Failed to establish WebSocket connection:', error);
    }
    return () => {
      this.wsConnection?.close();
      this.wsConnection = null;
    };
  }

  getConfig(): VibeKanbanConfig {
    return { ...this.config };
  }
}

let clientInstance: VibeKanbanClient | null = null;

export function getVibeKanbanClient(): VibeKanbanClient {
  if (!clientInstance) {
    clientInstance = new VibeKanbanClient();
  }
  return clientInstance;
}

export default VibeKanbanClient;
