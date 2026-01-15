/**
 * Vibe Kanban Integration Service
 * 
 * This service connects the minimalist_sidebar to the existing vibe-kanban application.
 * Vibe-kanban is a complete Rust/Axum backend + React frontend application for AI coding agents.
 */

import { Platform } from 'react-native';

// Configuration for vibe-kanban service
export interface VibeKanbanConfig {
  // Base URL for the vibe-kanban backend API
  apiBaseUrl: string;
  // WebSocket URL for real-time updates
  wsBaseUrl: string;
  // Frontend URL for embedding/linking
  frontendUrl: string;
}

// Get config from environment or use defaults
export function getVibeKanbanConfig(): VibeKanbanConfig {
  const isWeb = Platform.OS === 'web';
  
  // Default ports based on vibe-kanban configuration
  // In dev mode: Frontend on 3000, Backend on 3001 (PORT+1)
  // The backend auto-assigns if no PORT is set
  const defaultBackendPort = process.env.VIBE_KANBAN_BACKEND_PORT || '3001';
  const defaultFrontendPort = process.env.VIBE_KANBAN_FRONTEND_PORT || '3000';
  const host = process.env.VIBE_KANBAN_HOST || (isWeb ? window.location.hostname : '127.0.0.1');
  
  return {
    apiBaseUrl: process.env.VIBE_KANBAN_API_URL || `http://${host}:${defaultBackendPort}`,
    wsBaseUrl: process.env.VIBE_KANBAN_WS_URL || `ws://${host}:${defaultBackendPort}`,
    frontendUrl: process.env.VIBE_KANBAN_FRONTEND_URL || `http://${host}:${defaultFrontendPort}`,
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

/**
 * Vibe Kanban API Client
 */
export class VibeKanbanClient {
  private config: VibeKanbanConfig;
  private wsConnection: WebSocket | null = null;

  constructor(config?: Partial<VibeKanbanConfig>) {
    this.config = { ...getVibeKanbanConfig(), ...config };
  }

  /**
   * Check if vibe-kanban backend is available
   */
  async isAvailable(): Promise<boolean> {
    try {
      const response = await fetch(`${this.config.apiBaseUrl}/api/health`, {
        method: 'GET',
        headers: { 'Accept': 'application/json' },
      });
      return response.ok;
    } catch (error) {
      console.warn('Vibe-kanban backend not available:', error);
      return false;
    }
  }

  /**
   * Get the embed URL for vibe-kanban frontend
   */
  getEmbedUrl(): string {
    return this.config.frontendUrl;
  }

  /**
   * Fetch projects from vibe-kanban
   */
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

  /**
   * Get tasks for a project
   */
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

  /**
   * Connect to real-time WebSocket stream
   */
  connectToStream(
    endpoint: string,
    onMessage: (data: any) => void,
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
      
      this.wsConnection.onerror = (error) => {
        console.error('WebSocket error:', error);
        onError?.(error);
      };
      
      this.wsConnection.onclose = () => {
        console.log('WebSocket connection closed');
      };
    } catch (error) {
      console.error('Failed to establish WebSocket connection:', error);
    }
    
    // Return cleanup function
    return () => {
      if (this.wsConnection) {
        this.wsConnection.close();
        this.wsConnection = null;
      }
    };
  }

  /**
   * Get configuration for display
   */
  getConfig(): VibeKanbanConfig {
    return { ...this.config };
  }
}

// Singleton instance
let clientInstance: VibeKanbanClient | null = null;

export function getVibeKanbanClient(): VibeKanbanClient {
  if (!clientInstance) {
    clientInstance = new VibeKanbanClient();
  }
  return clientInstance;
}

export default VibeKanbanClient;
