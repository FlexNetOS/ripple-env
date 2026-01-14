import { NotificationBadge } from '@/contexts/sidebar-context';

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
    id: 'nodes',
    icon: 'network',
    label: 'Compute Nodes',
    badge: { status: 'success' as const },
    workspace: 'infrastructure' as const,
  },
  {
    id: 'storage',
    icon: 'database',
    label: 'Storage',
    workspace: 'infrastructure' as const,
  },
  {
    id: 'logs',
    icon: 'file-text',
    label: 'Logs',
    badge: { count: 12 },
    workspace: 'devops' as const,
  },
  {
    id: 'analytics',
    icon: 'chart',
    label: 'Analytics',
    workspace: 'all' as const,
  },
  {
    id: 'settings',
    icon: 'settings',
    label: 'Settings',
    workspace: 'settings' as const,
    shortcut: '⌘,',
  },
];

// Sidebar Content Map
export const contentMap: Record<string, SidebarContent> = {
  // AI COMMAND CENTER - The brain that runs everything
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
            hasDropdown: true,
            children: [
              { id: 'morning', icon: 'clock', label: 'Morning routine optimization', metadata: '2m ago' },
              { id: 'schedule', icon: 'calendar', label: 'Family schedule coordination', metadata: '5m ago' },
            ],
          },
          {
            id: 'work-strategy',
            icon: 'message-square',
            label: 'Work Strategy',
            metadata: 'Project planning',
            hasDropdown: true,
            children: [
              { id: 'q1-goals', icon: 'target', label: 'Q1 goals review', metadata: 'Active now' },
              { id: 'team-perf', icon: 'users', label: 'Team performance insights', metadata: '1h ago' },
            ],
          },
          {
            id: 'home-automation',
            icon: 'message-square',
            label: 'Home Automation',
            metadata: 'Managing smart home',
            hasDropdown: true,
            children: [
              { id: 'evening', icon: 'moon', label: 'Evening scene setup', metadata: 'Scheduled 6 PM' },
              { id: 'energy', icon: 'zap', label: 'Energy optimization tips', metadata: 'Today' },
            ],
          },
        ],
      },
      {
        title: 'AI Knowledge Base',
        items: [
          {
            id: 'goals',
            icon: 'target',
            label: 'Goals & Objectives',
            badge: { count: 8 },
            hasDropdown: true,
            children: [
              { id: 'q1-launch', icon: 'rocket', label: 'Complete Q1 product launch', status: 'good', metadata: '85% complete' },
              { id: 'vacation', icon: 'plane', label: 'Family vacation planning', status: 'warning', metadata: 'Needs attention' },
              { id: 'security', icon: 'shield', label: 'Home security upgrade', status: 'good', metadata: 'On track' },
              { id: 'health', icon: 'heart', label: 'Health & fitness targets', status: 'good', metadata: 'Meeting goals' },
            ],
          },
          {
            id: 'ideas',
            icon: 'lightbulb',
            label: 'Ideas & Insights',
            badge: { count: 12 },
            hasDropdown: true,
            children: [
              { id: 'weekend', icon: 'sparkles', label: 'Weekend automation ideas', metadata: 'AI suggested' },
              { id: 'cost', icon: 'dollar', label: 'Cost-saving strategies', metadata: 'High priority' },
              { id: 'family', icon: 'users', label: 'Family time optimization', metadata: 'New' },
            ],
          },
          {
            id: 'rules',
            icon: 'shield',
            label: 'Rules & Protocols',
            hasDropdown: true,
            children: [
              { id: 'work-life', icon: 'balance', label: 'Work-life balance rules', status: 'good' },
              { id: 'safety', icon: 'shield-check', label: 'Family safety protocols', status: 'good' },
              { id: 'security-rules', icon: 'lock', label: 'Security guidelines', status: 'good' },
              { id: 'energy-rules', icon: 'zap', label: 'Energy efficiency rules', status: 'good' },
            ],
          },
          {
            id: 'memory',
            icon: 'brain',
            label: 'Memory & Context',
            hasDropdown: true,
            children: [
              { id: 'prefs', icon: 'sliders', label: 'Preferences & habits', metadata: '128 entries' },
              { id: 'relationships', icon: 'users', label: 'Relationship context', metadata: 'Family & work' },
              { id: 'decisions', icon: 'git-branch', label: 'Decision history', metadata: 'Last 90 days' },
              { id: 'patterns', icon: 'activity', label: 'Life patterns', metadata: 'AI analyzed' },
            ],
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
            children: [
              { id: 'voice', icon: 'mic', label: 'Voice commands', status: 'good' },
              { id: 'visual', icon: 'eye', label: 'Visual recognition', status: 'good' },
              { id: 'text', icon: 'type', label: 'Text generation', status: 'good' },
              { id: 'translate', icon: 'globe', label: 'Real-time translation', status: 'good' },
            ],
          },
          {
            id: 'assistants',
            icon: 'robot',
            label: 'Specialized Assistants',
            hasDropdown: true,
            children: [
              { id: 'productivity', icon: 'briefcase', label: 'Work productivity coach', status: 'online' },
              { id: 'coordinator', icon: 'users', label: 'Family coordinator', status: 'online' },
              { id: 'health-advisor', icon: 'heart', label: 'Health advisor', status: 'online' },
              { id: 'financial', icon: 'dollar', label: 'Financial planner', status: 'online' },
              { id: 'home-expert', icon: 'home', label: 'Home automation expert', status: 'online' },
            ],
          },
          {
            id: 'workflows',
            icon: 'workflow',
            label: 'Automation & Workflows',
            badge: { count: 15, status: 'success' },
            hasDropdown: true,
            children: [
              { id: 'morning-routine', icon: 'sun', label: 'Morning routine', status: 'good', metadata: 'Active' },
              { id: 'work-mode', icon: 'briefcase', label: 'Work mode trigger', status: 'good', metadata: 'Active' },
              { id: 'evening-routine', icon: 'moon', label: 'Evening wind-down', status: 'good', metadata: 'Active' },
              { id: 'weekend', icon: 'calendar', label: 'Weekend automation', status: 'good', metadata: 'Active' },
            ],
          },
        ],
      },
    ],
  },

  // BUILD PIPELINE
  build: {
    title: 'Build Pipeline',
    headerBadge: { count: 2, status: 'success' },
    sections: [
      {
        title: 'Active Builds',
        items: [
          {
            id: 'p3-build',
            icon: 'play',
            label: 'P3 - Inference & State',
            badge: { status: 'info', pulse: true },
            metadata: 'Running • 45%',
            hasDropdown: true,
            children: [
              { id: 'p3-step1', icon: 'check', label: 'LocalAI deployment', status: 'good', metadata: 'Complete' },
              { id: 'p3-step2', icon: 'loader', label: 'Model loading', status: 'busy', metadata: 'In progress' },
              { id: 'p3-step3', icon: 'clock', label: 'State sync', metadata: 'Pending' },
            ],
          },
        ],
      },
      {
        title: 'Build Phases',
        items: [
          { id: 'p0', icon: 'check-circle', label: 'P0 - Environment Foundation', status: 'good', metadata: 'Complete' },
          { id: 'p1', icon: 'check-circle', label: 'P1 - Vendor Integration', status: 'good', metadata: 'Complete' },
          { id: 'p2', icon: 'check-circle', label: 'P2 - Model Cluster', status: 'good', metadata: 'Complete' },
          { id: 'p3', icon: 'loader', label: 'P3 - Inference & State', status: 'busy', metadata: '45%' },
          { id: 'p4', icon: 'clock', label: 'P4 - Database & Inference', metadata: 'Pending' },
          { id: 'p5', icon: 'clock', label: 'P5 - Orchestration', metadata: 'Pending' },
          { id: 'p6', icon: 'clock', label: 'P6 - Testing & Promotion', metadata: 'Pending' },
          { id: 'p7', icon: 'clock', label: 'P7 - Build & Package', metadata: 'Pending' },
        ],
      },
      {
        title: 'Quick Actions',
        items: [
          { id: 'start-build', icon: 'play', label: 'Start New Build' },
          { id: 'view-logs', icon: 'file-text', label: 'View Build Logs' },
          { id: 'rollback', icon: 'rotate-ccw', label: 'Rollback Last Build' },
        ],
      },
    ],
  },

  // AGENTS
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
          { id: 'qwen', icon: 'hammer', label: 'Qwen-Builder', status: 'busy', metadata: 'Building • 3 jobs' },
          { id: 'gemma', icon: 'check-circle', label: 'Gemma-Validator', status: 'online', metadata: 'Validating • Idle' },
        ],
      },
      {
        title: 'MOE Policy',
        items: [
          {
            id: 'routing',
            icon: 'git-branch',
            label: 'Routing Strategy',
            hasDropdown: true,
            children: [
              { id: 'load-balance', icon: 'scale', label: 'Load Balancing', status: 'good', metadata: 'Active' },
              { id: 'priority', icon: 'arrow-up', label: 'Priority Queue', status: 'good', metadata: 'Enabled' },
              { id: 'fallback', icon: 'shield', label: 'Fallback Chain', status: 'good', metadata: 'Configured' },
            ],
          },
          {
            id: 'scaling',
            icon: 'trending-up',
            label: 'Auto-Scaling',
            hasDropdown: true,
            children: [
              { id: 'min-agents', icon: 'minus', label: 'Min Agents: 3', metadata: 'Current: 5' },
              { id: 'max-agents', icon: 'plus', label: 'Max Agents: 10', metadata: 'Limit set' },
              { id: 'scale-trigger', icon: 'zap', label: 'Scale Trigger: 80% CPU', status: 'good' },
            ],
          },
        ],
      },
    ],
  },

  // INFRASTRUCTURE
  infra: {
    title: 'Infrastructure',
    headerBadge: { status: 'success' },
    sections: [
      {
        title: 'Compute Nodes',
        items: [
          { id: 'master', icon: 'server', label: 'ripple-master', status: 'good', metadata: 'CPU 45% • RAM 62%' },
          { id: 'worker1', icon: 'server', label: 'ripple-worker-01', status: 'good', metadata: 'CPU 78% • RAM 85%' },
          { id: 'worker2', icon: 'server', label: 'ripple-worker-02', status: 'good', metadata: 'CPU 56% • RAM 71%' },
          { id: 'gpu1', icon: 'cpu', label: 'ripple-gpu-01', status: 'good', metadata: 'GPU 92% • VRAM 14GB' },
        ],
      },
      {
        title: 'Services',
        items: [
          { id: 'holochain', icon: 'link', label: 'Holochain Conductor', status: 'good', metadata: 'Running' },
          { id: 'qdrant', icon: 'database', label: 'Qdrant Vector DB', status: 'good', metadata: 'Healthy' },
          { id: 'redis', icon: 'database', label: 'Redis Cache', status: 'good', metadata: 'Connected' },
          { id: 'postgres', icon: 'database', label: 'PostgreSQL', status: 'good', metadata: 'Active' },
        ],
      },
      {
        title: 'Network',
        items: [
          { id: 'ingress', icon: 'arrow-down', label: 'Ingress', status: 'good', metadata: '1.2 GB/s' },
          { id: 'egress', icon: 'arrow-up', label: 'Egress', status: 'good', metadata: '0.8 GB/s' },
          { id: 'latency', icon: 'clock', label: 'Avg Latency', status: 'good', metadata: '12ms' },
        ],
      },
    ],
  },

  // SETTINGS
  settings: {
    title: 'Settings',
    sections: [
      {
        title: 'General',
        items: [
          { id: 'appearance', icon: 'palette', label: 'Appearance', metadata: 'Dark mode' },
          { id: 'notifications', icon: 'bell', label: 'Notifications', metadata: 'Enabled' },
          { id: 'language', icon: 'globe', label: 'Language', metadata: 'English' },
        ],
      },
      {
        title: 'System',
        items: [
          { id: 'env', icon: 'terminal', label: 'Environment Variables' },
          { id: 'api-keys', icon: 'key', label: 'API Keys' },
          { id: 'integrations', icon: 'plug', label: 'Integrations' },
        ],
      },
      {
        title: 'About',
        items: [
          { id: 'version', icon: 'info', label: 'Version', metadata: 'v1.0.0' },
          { id: 'docs', icon: 'book', label: 'Documentation' },
          { id: 'support', icon: 'help-circle', label: 'Support' },
        ],
      },
    ],
  },
};

export function getSidebarContent(section: string): SidebarContent {
  return contentMap[section] || contentMap.ai;
}
