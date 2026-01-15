/**
 * Sidebar Navigation Data
 * Source of truth: minimalist_sidebar_react
 * Synced for Vue/Vite variant
 */

export interface NotificationBadge {
  count?: number;
  status?: 'info' | 'success' | 'warning' | 'error';
  pulse?: boolean;
}

export interface MenuItem {
  id: string;
  icon: string;
  label: string;
  hasDropdown?: boolean;
  isActive?: boolean;
  status?: 'online' | 'offline' | 'away' | 'busy' | 'good' | 'warning' | 'error';
  badge?: NotificationBadge;
  children?: MenuItem[];
  metadata?: string;
  shortcut?: string;
}

export interface MenuSection {
  title: string;
  items: MenuItem[];
}

export interface SidebarContent {
  title: string;
  sections: MenuSection[];
  headerBadge?: NotificationBadge;
}

// Icon Rail Navigation Items
export const navItems = [
  {
    id: 'ai',
    icon: 'robot',
    label: 'AI Command Center',
    badge: { count: 3, status: 'info' as const },
    workspace: 'all' as const,
    shortcut: '⌘1',
  },
  {
    id: 'build',
    icon: 'hammer',
    label: 'Build Pipeline',
    badge: { count: 2, status: 'success' as const },
    workspace: 'devops' as const,
    shortcut: '⌘B',
  },
  {
    id: 'agents',
    icon: 'users',
    label: 'Agents',
    badge: { count: 5, status: 'info' as const },
    workspace: 'agents' as const,
    shortcut: '⌘A',
  },
  {
    id: 'infra',
    icon: 'server',
    label: 'Infrastructure',
    badge: { status: 'success' as const },
    workspace: 'infrastructure' as const,
    shortcut: '⌘I',
  },
  {
    id: 'kanban',
    icon: 'kanban',
    label: 'Vibe Kanban',
    badge: { status: 'info' as const },
    workspace: 'tasks' as const,
    shortcut: '⌘K',
  },
  {
    id: 'settings',
    icon: 'settings',
    label: 'Settings',
    workspace: 'settings' as const,
    shortcut: '⌘,',
  },
];

// Content Map for detail panels
export const contentMap: Record<string, SidebarContent> = {
  ai: {
    title: 'AI Command Center',
    headerBadge: { count: 3, status: 'info' },
    sections: [
      {
        title: 'Active Conversations',
        items: [
          {
            id: 'life-assistant',
            icon: 'message-square',
            label: 'Life Assistant',
            badge: { count: 2, pulse: true },
            metadata: 'Running your day',
          },
          {
            id: 'work-strategy',
            icon: 'message-square',
            label: 'Work Strategy',
            metadata: 'Project planning',
          },
        ],
      },
      {
        title: 'AI Capabilities',
        items: [
          {
            id: 'multi-modal',
            icon: 'sparkles',
            label: 'Multi-Modal Interface',
            hasDropdown: true,
          },
          {
            id: 'assistants',
            icon: 'robot',
            label: 'Specialized Assistants',
            hasDropdown: true,
          },
        ],
      },
    ],
  },
  build: {
    title: 'Build Pipeline',
    headerBadge: { count: 2, status: 'success' },
    sections: [
      {
        title: 'Build Phases',
        items: [
          { id: 'p0', icon: 'check-circle', label: 'P0 - Environment Foundation', status: 'good', metadata: 'Complete' },
          { id: 'p1', icon: 'check-circle', label: 'P1 - Vendor Integration', status: 'good', metadata: 'Complete' },
          { id: 'p2', icon: 'check-circle', label: 'P2 - Model Cluster', status: 'good', metadata: 'Complete' },
          { id: 'p3', icon: 'loader', label: 'P3 - Inference & State', status: 'busy', metadata: '45%' },
        ],
      },
    ],
  },
  agents: {
    title: 'Agents',
    headerBadge: { count: 5, status: 'info' },
    sections: [
      {
        title: 'Active Agents',
        items: [
          { id: 'agixt', icon: 'robot', label: 'AGiXT-01', status: 'online', metadata: 'Orchestrator • 12 tasks' },
          { id: 'localai', icon: 'cpu', label: 'LocalAI-Primary', status: 'online', metadata: 'Inference • GPU 78%' },
          { id: 'deepseek', icon: 'brain', label: 'DeepSeek-R1', status: 'online', metadata: 'Reasoning • Active' },
        ],
      },
    ],
  },
  infra: {
    title: 'Infrastructure',
    headerBadge: { status: 'success' },
    sections: [
      {
        title: 'Compute Nodes',
        items: [
          { id: 'master', icon: 'server', label: 'ripple-master', status: 'good', metadata: 'CPU 45% • RAM 62%' },
          { id: 'worker1', icon: 'server', label: 'ripple-worker-01', status: 'good', metadata: 'CPU 78% • RAM 85%' },
          { id: 'gpu1', icon: 'cpu', label: 'ripple-gpu-01', status: 'good', metadata: 'GPU 92% • VRAM 14GB' },
        ],
      },
    ],
  },
  kanban: {
    title: 'Vibe Kanban',
    headerBadge: { status: 'info' },
    sections: [
      {
        title: 'Projects',
        items: [
          { id: 'vibe-projects', icon: 'folder', label: 'View Projects', metadata: 'Browse all projects' },
          { id: 'vibe-tasks', icon: 'check-square', label: 'Active Tasks', metadata: 'Current work' },
        ],
      },
    ],
  },
  settings: {
    title: 'Settings',
    sections: [
      {
        title: 'General',
        items: [
          { id: 'appearance', icon: 'palette', label: 'Appearance', metadata: 'Dark mode' },
          { id: 'notifications', icon: 'bell', label: 'Notifications', metadata: 'Enabled' },
        ],
      },
    ],
  },
};

export function getSidebarContent(section: string): SidebarContent {
  return contentMap[section] || contentMap.ai;
}
