/**
 * Real-time data service for Ripple DevOps platform
 * Provides simulated real-time updates for agents, builds, and infrastructure
 */

export interface AgentStatus {
  id: string;
  name: string;
  type: 'orchestrator' | 'validator' | 'inference' | 'storage' | 'network';
  status: 'active' | 'idle' | 'busy' | 'error' | 'offline';
  cpuUsage: number;
  memoryUsage: number;
  tasksCompleted: number;
  tasksQueued: number;
  lastHeartbeat: Date;
  uptime: number; // seconds
}

export interface BuildPhase {
  id: string;
  name: string;
  status: 'pending' | 'running' | 'completed' | 'failed' | 'skipped';
  progress: number; // 0-100
  startTime?: Date;
  endTime?: Date;
  logs: string[];
}

export interface BuildStatus {
  id: string;
  version: string;
  currentPhase: number;
  totalPhases: number;
  phases: BuildPhase[];
  overallProgress: number;
  status: 'idle' | 'building' | 'testing' | 'deploying' | 'completed' | 'failed';
  startTime?: Date;
  estimatedCompletion?: Date;
}

export interface InfraNode {
  id: string;
  name: string;
  type: 'compute' | 'storage' | 'inference' | 'gateway';
  status: 'healthy' | 'degraded' | 'critical' | 'offline';
  cpuUsage: number;
  memoryUsage: number;
  diskUsage: number;
  networkIn: number; // MB/s
  networkOut: number; // MB/s
  location: string;
  peers: number;
}

export interface SystemHealth {
  overall: 'healthy' | 'degraded' | 'critical';
  activeAgents: number;
  totalAgents: number;
  runningTasks: number;
  queuedTasks: number;
  buildStatus: 'idle' | 'building' | 'testing' | 'deploying' | 'completed' | 'failed';
  currentPhase: string;
  nodesOnline: number;
  totalNodes: number;
  alerts: Alert[];
}

export interface Alert {
  id: string;
  severity: 'info' | 'warning' | 'error' | 'critical';
  title: string;
  message: string;
  timestamp: Date;
  source: string;
  acknowledged: boolean;
}

// Simulated data generators
function randomBetween(min: number, max: number): number {
  return Math.floor(Math.random() * (max - min + 1)) + min;
}

function randomFloat(min: number, max: number, decimals: number = 1): number {
  const value = Math.random() * (max - min) + min;
  return parseFloat(value.toFixed(decimals));
}

// Initial mock data
const mockAgents: AgentStatus[] = [
  {
    id: 'agent-001',
    name: 'Chief Orchestrator',
    type: 'orchestrator',
    status: 'active',
    cpuUsage: 45,
    memoryUsage: 62,
    tasksCompleted: 1247,
    tasksQueued: 12,
    lastHeartbeat: new Date(),
    uptime: 86400,
  },
  {
    id: 'agent-002',
    name: 'Validator Alpha',
    type: 'validator',
    status: 'busy',
    cpuUsage: 78,
    memoryUsage: 54,
    tasksCompleted: 892,
    tasksQueued: 5,
    lastHeartbeat: new Date(),
    uptime: 72000,
  },
  {
    id: 'agent-003',
    name: 'Inference Engine',
    type: 'inference',
    status: 'active',
    cpuUsage: 92,
    memoryUsage: 88,
    tasksCompleted: 3421,
    tasksQueued: 23,
    lastHeartbeat: new Date(),
    uptime: 43200,
  },
  {
    id: 'agent-004',
    name: 'Storage Manager',
    type: 'storage',
    status: 'idle',
    cpuUsage: 12,
    memoryUsage: 34,
    tasksCompleted: 567,
    tasksQueued: 0,
    lastHeartbeat: new Date(),
    uptime: 129600,
  },
  {
    id: 'agent-005',
    name: 'Network Coordinator',
    type: 'network',
    status: 'active',
    cpuUsage: 34,
    memoryUsage: 41,
    tasksCompleted: 2134,
    tasksQueued: 8,
    lastHeartbeat: new Date(),
    uptime: 64800,
  },
];

const buildPhases: BuildPhase[] = [
  { id: 'P0', name: 'Environment Foundation', status: 'completed', progress: 100, logs: ['NixOS flake initialized', 'Dependencies resolved'] },
  { id: 'P1', name: 'Core Infrastructure', status: 'completed', progress: 100, logs: ['Holochain conductor ready', 'DHT network established'] },
  { id: 'P2', name: 'Agent Framework', status: 'completed', progress: 100, logs: ['MOE policies loaded', 'Agent registry initialized'] },
  { id: 'P3', name: 'Intelligence Layer', status: 'running', progress: 67, logs: ['Loading inference models...', 'Vector DB connecting...'] },
  { id: 'P4', name: 'Integration', status: 'pending', progress: 0, logs: [] },
  { id: 'P5', name: 'Testing', status: 'pending', progress: 0, logs: [] },
  { id: 'P6', name: 'Optimization', status: 'pending', progress: 0, logs: [] },
  { id: 'P7', name: 'Deployment', status: 'pending', progress: 0, logs: [] },
];

const mockNodes: InfraNode[] = [
  { id: 'node-001', name: 'Compute Primary', type: 'compute', status: 'healthy', cpuUsage: 67, memoryUsage: 72, diskUsage: 45, networkIn: 125.4, networkOut: 89.2, location: 'us-east-1', peers: 12 },
  { id: 'node-002', name: 'Inference GPU', type: 'inference', status: 'healthy', cpuUsage: 94, memoryUsage: 88, diskUsage: 34, networkIn: 234.1, networkOut: 156.8, location: 'us-west-2', peers: 8 },
  { id: 'node-003', name: 'Vector Storage', type: 'storage', status: 'degraded', cpuUsage: 23, memoryUsage: 89, diskUsage: 87, networkIn: 45.2, networkOut: 123.4, location: 'eu-west-1', peers: 15 },
  { id: 'node-004', name: 'API Gateway', type: 'gateway', status: 'healthy', cpuUsage: 34, memoryUsage: 45, diskUsage: 23, networkIn: 456.7, networkOut: 389.2, location: 'ap-south-1', peers: 24 },
];

const mockAlerts: Alert[] = [
  { id: 'alert-001', severity: 'warning', title: 'High Memory Usage', message: 'Inference node memory at 88%', timestamp: new Date(Date.now() - 300000), source: 'node-002', acknowledged: false },
  { id: 'alert-002', severity: 'info', title: 'Build Phase Completed', message: 'P2 Agent Framework completed successfully', timestamp: new Date(Date.now() - 600000), source: 'build-system', acknowledged: true },
  { id: 'alert-003', severity: 'error', title: 'Storage Warning', message: 'Vector DB disk usage at 87%', timestamp: new Date(Date.now() - 900000), source: 'node-003', acknowledged: false },
];

// Service class for real-time data
class RealtimeDataService {
  private agents: AgentStatus[] = [...mockAgents];
  private build: BuildStatus = {
    id: 'build-001',
    version: '1.0.0-alpha',
    currentPhase: 3,
    totalPhases: 8,
    phases: [...buildPhases],
    overallProgress: 45,
    status: 'building',
    startTime: new Date(Date.now() - 3600000),
    estimatedCompletion: new Date(Date.now() + 7200000),
  };
  private nodes: InfraNode[] = [...mockNodes];
  private alerts: Alert[] = [...mockAlerts];
  private listeners: Map<string, Set<(data: any) => void>> = new Map();

  // Subscribe to data updates
  subscribe(channel: 'agents' | 'build' | 'nodes' | 'health' | 'alerts', callback: (data: any) => void): () => void {
    if (!this.listeners.has(channel)) {
      this.listeners.set(channel, new Set());
    }
    this.listeners.get(channel)!.add(callback);

    // Return unsubscribe function
    return () => {
      this.listeners.get(channel)?.delete(callback);
    };
  }

  // Emit updates to subscribers
  private emit(channel: string, data: any) {
    this.listeners.get(channel)?.forEach(callback => callback(data));
  }

  // Get current agent statuses
  getAgents(): AgentStatus[] {
    return this.agents;
  }

  // Get current build status
  getBuildStatus(): BuildStatus {
    return this.build;
  }

  // Get infrastructure nodes
  getNodes(): InfraNode[] {
    return this.nodes;
  }

  // Get system health overview
  getSystemHealth(): SystemHealth {
    const activeAgents = this.agents.filter(a => a.status !== 'offline' && a.status !== 'error').length;
    const runningTasks = this.agents.reduce((sum, a) => sum + (a.status === 'busy' ? 1 : 0), 0);
    const queuedTasks = this.agents.reduce((sum, a) => sum + a.tasksQueued, 0);
    const nodesOnline = this.nodes.filter(n => n.status !== 'offline').length;
    
    const hasErrors = this.nodes.some(n => n.status === 'critical') || this.agents.some(a => a.status === 'error');
    const hasDegraded = this.nodes.some(n => n.status === 'degraded');

    return {
      overall: hasErrors ? 'critical' : hasDegraded ? 'degraded' : 'healthy',
      activeAgents,
      totalAgents: this.agents.length,
      runningTasks,
      queuedTasks,
      buildStatus: this.build.status,
      currentPhase: this.build.phases[this.build.currentPhase]?.name || 'Unknown',
      nodesOnline,
      totalNodes: this.nodes.length,
      alerts: this.alerts.filter(a => !a.acknowledged),
    };
  }

  // Get alerts
  getAlerts(): Alert[] {
    return this.alerts;
  }

  // Acknowledge an alert
  acknowledgeAlert(alertId: string) {
    const alert = this.alerts.find(a => a.id === alertId);
    if (alert) {
      alert.acknowledged = true;
      this.emit('alerts', this.alerts);
    }
  }

  // Simulate real-time updates (call this periodically)
  simulateUpdate() {
    // Update agent metrics
    this.agents = this.agents.map(agent => ({
      ...agent,
      cpuUsage: Math.min(100, Math.max(5, agent.cpuUsage + randomBetween(-5, 5))),
      memoryUsage: Math.min(100, Math.max(10, agent.memoryUsage + randomBetween(-3, 3))),
      tasksCompleted: agent.tasksCompleted + (agent.status === 'busy' ? randomBetween(0, 2) : 0),
      tasksQueued: Math.max(0, agent.tasksQueued + randomBetween(-1, 2)),
      lastHeartbeat: new Date(),
    }));

    // Update build progress
    if (this.build.status === 'building') {
      const currentPhase = this.build.phases[this.build.currentPhase];
      if (currentPhase && currentPhase.status === 'running') {
        currentPhase.progress = Math.min(100, currentPhase.progress + randomBetween(1, 5));
        if (currentPhase.progress >= 100) {
          currentPhase.status = 'completed';
          currentPhase.endTime = new Date();
          if (this.build.currentPhase < this.build.totalPhases - 1) {
            this.build.currentPhase++;
            const nextPhase = this.build.phases[this.build.currentPhase];
            if (nextPhase) {
              nextPhase.status = 'running';
              nextPhase.startTime = new Date();
            }
          } else {
            this.build.status = 'completed';
          }
        }
      }
      this.build.overallProgress = Math.round(
        this.build.phases.reduce((sum, p) => sum + p.progress, 0) / this.build.totalPhases
      );
    }

    // Update node metrics
    this.nodes = this.nodes.map(node => ({
      ...node,
      cpuUsage: Math.min(100, Math.max(5, node.cpuUsage + randomBetween(-8, 8))),
      memoryUsage: Math.min(100, Math.max(10, node.memoryUsage + randomBetween(-5, 5))),
      networkIn: randomFloat(10, 500),
      networkOut: randomFloat(10, 400),
    }));

    // Emit updates
    this.emit('agents', this.agents);
    this.emit('build', this.build);
    this.emit('nodes', this.nodes);
    this.emit('health', this.getSystemHealth());
  }

  // Start automatic updates
  startAutoUpdate(intervalMs: number = 3000): () => void {
    const interval = setInterval(() => this.simulateUpdate(), intervalMs);
    return () => clearInterval(interval);
  }
}

// Singleton instance
export const realtimeData = new RealtimeDataService();

// React hook for real-time data
import { useState, useEffect } from 'react';

export function useRealtimeAgents(): AgentStatus[] {
  const [agents, setAgents] = useState<AgentStatus[]>(realtimeData.getAgents());

  useEffect(() => {
    const unsubscribe = realtimeData.subscribe('agents', setAgents);
    return unsubscribe;
  }, []);

  return agents;
}

export function useRealtimeBuild(): BuildStatus {
  const [build, setBuild] = useState<BuildStatus>(realtimeData.getBuildStatus());

  useEffect(() => {
    const unsubscribe = realtimeData.subscribe('build', setBuild);
    return unsubscribe;
  }, []);

  return build;
}

export function useRealtimeNodes(): InfraNode[] {
  const [nodes, setNodes] = useState<InfraNode[]>(realtimeData.getNodes());

  useEffect(() => {
    const unsubscribe = realtimeData.subscribe('nodes', setNodes);
    return unsubscribe;
  }, []);

  return nodes;
}

export function useSystemHealth(): SystemHealth {
  const [health, setHealth] = useState<SystemHealth>(realtimeData.getSystemHealth());

  useEffect(() => {
    const unsubscribe = realtimeData.subscribe('health', setHealth);
    return unsubscribe;
  }, []);

  return health;
}

export function useAlerts(): Alert[] {
  const [alerts, setAlerts] = useState<Alert[]>(realtimeData.getAlerts());

  useEffect(() => {
    const unsubscribe = realtimeData.subscribe('alerts', setAlerts);
    return unsubscribe;
  }, []);

  return alerts;
}
