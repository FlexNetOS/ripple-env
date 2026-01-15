// Complete sidebar panel data - adapted from minimalist_sidebar_react
// Each panel corresponds to an icon in the icon rail

export type WorkspaceType = 'all' | 'productivity' | 'communication' | 'intelligence' | 'infrastructure' | 'life' | 'business';

export interface WorkspaceOption {
  id: WorkspaceType;
  label: string;
  icon: string;
}

export const WORKSPACES: WorkspaceOption[] = [
  { id: 'all', label: 'All', icon: 'Layers' },
  { id: 'productivity', label: 'Work', icon: 'Briefcase' },
  { id: 'communication', label: 'People', icon: 'Users' },
  { id: 'intelligence', label: 'AI', icon: 'Bot' },
  { id: 'infrastructure', label: 'Tech', icon: 'Cpu' },
  { id: 'life', label: 'Life', icon: 'Heart' },
  { id: 'business', label: 'Business', icon: 'Building' },
];

export interface MenuItem {
  id: string;
  label: string;
  icon: string;
  badge?: number;
  badgeColor?: string;
  metadata?: string;
  shortcut?: string;
  status?: 'online' | 'offline' | 'busy' | 'away';
  route?: string;
}

export interface MenuSection {
  id: string;
  title: string;
  items: MenuItem[];
  collapsible?: boolean;
  defaultExpanded?: boolean;
}

export interface SidebarPanel {
  id: string;
  title: string;
  icon: string;
  badge?: number;
  badgeColor?: string;
  sections: MenuSection[];
  workspaces: WorkspaceType[];
}

// Section routes mapping
export const SECTION_ROUTES: Record<string, string> = {
  'ai-command-center': '/chat',
  'life-dashboard': '/dashboard',
  'contacts': '/dashboard',
  'wallet': '/billing',
  'legal-compliance': '/dashboard',
  'tasks': '/dashboard',
  'projects': '/dashboard',
  'calendar': '/dashboard',
  'files': '/files',
  'analytics-hub': '/dashboard',
  'social-media': '/dashboard',
  'compute-nodes': '/dashboard',
  'notifications': '/notifications',
  'favorites': '/dashboard',
  'settings': '/settings',
  'profile': '/settings',
};

// Panel 1: AI Command Center
export const AI_COMMAND_CENTER: SidebarPanel = {
  id: 'ai-command-center',
  title: 'AI Command Center',
  icon: 'Layers',
  badge: 3,
  badgeColor: '#22C55E',
  workspaces: ['all', 'intelligence', 'productivity', 'infrastructure'],
  sections: [
    {
      id: 'active-conversations',
      title: 'ACTIVE CONVERSATIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'life-assistant', label: 'Life Assistant', icon: 'MessageSquare', metadata: 'Running your day', badge: 2, badgeColor: '#EF4444', route: '/chat' },
        { id: 'work-strategy', label: 'Work Strategy', icon: 'MessageSquare', metadata: 'Project planning', route: '/chat' },
        { id: 'home-automation', label: 'Home Automation', icon: 'MessageSquare', metadata: 'Managing smart home', route: '/chat' },
      ],
    },
    {
      id: 'ai-knowledge-base',
      title: 'AI KNOWLEDGE BASE',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'goals-objectives', label: 'Goals & Objectives', icon: 'Target', badge: 8, badgeColor: '#EF4444' },
        { id: 'ideas-insights', label: 'Ideas & Insights', icon: 'Lightbulb', badge: 12, badgeColor: '#3B82F6' },
        { id: 'rules-protocols', label: 'Rules & Protocols', icon: 'Shield' },
        { id: 'memory-context', label: 'Memory & Context', icon: 'Brain' },
      ],
    },
    {
      id: 'ai-capabilities',
      title: 'AI CAPABILITIES',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'multi-modal', label: 'Multi-Modal Interface', icon: 'LayoutGrid' },
        { id: 'specialized-assistants', label: 'Specialized Assistants', icon: 'Bot' },
        { id: 'automation-workflows', label: 'Automation & Workflows', icon: 'Workflow', badge: 15, badgeColor: '#3B82F6' },
      ],
    },
  ],
};

// Panel 2: Life Dashboard
export const LIFE_DASHBOARD: SidebarPanel = {
  id: 'life-dashboard',
  title: 'Life Dashboard',
  icon: 'LayoutDashboard',
  workspaces: ['all', 'life', 'communication'],
  sections: [
    {
      id: 'overview',
      title: 'OVERVIEW',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'todays-summary', label: "Today's Summary", icon: 'CalendarCheck', metadata: 'All areas', route: '/dashboard' },
        { id: 'key-metrics', label: 'Key Metrics', icon: 'TrendingUp', route: '/dashboard' },
      ],
    },
    {
      id: 'domain-dashboards',
      title: 'DOMAIN DASHBOARDS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'work-productivity', label: 'Work & Productivity', icon: 'Briefcase', badge: 5, badgeColor: '#22C55E', route: '/dashboard' },
        { id: 'family-personal', label: 'Family & Personal', icon: 'Heart', badge: 2, badgeColor: '#3B82F6', route: '/dashboard' },
        { id: 'home-environment', label: 'Home & Environment', icon: 'Home', route: '/dashboard' },
        { id: 'analytics-hub', label: 'Analytics Hub', icon: 'BarChart2', route: '/dashboard' },
      ],
    },
    {
      id: 'recent-activity',
      title: 'RECENT ACTIVITY',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'activity-feed', label: 'Activity Feed', icon: 'Activity', badge: 8, badgeColor: '#EF4444', route: '/dashboard' },
      ],
    },
  ],
};

// Panel 3: Contacts
export const CONTACTS: SidebarPanel = {
  id: 'contacts',
  title: 'Contacts',
  icon: 'Users',
  badge: 7,
  badgeColor: '#22C55E',
  workspaces: ['all', 'communication', 'productivity', 'life'],
  sections: [
    {
      id: 'communication',
      title: 'COMMUNICATION',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'video-conferencing', label: 'Video Conferencing', icon: 'Video', badge: 1, badgeColor: '#F59E0B' },
        { id: 'calls', label: 'Calls', icon: 'Phone', badge: 2, badgeColor: '#EF4444' },
        { id: 'messaging', label: 'Messaging', icon: 'MessageCircle', badge: 5, badgeColor: '#22C55E' },
      ],
    },
    {
      id: 'work-contacts',
      title: 'WORK CONTACTS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'dev-team', label: 'Development Team', icon: 'Users', badge: 4, badgeColor: '#3B82F6' },
        { id: 'design-team', label: 'Design Team', icon: 'Users', badge: 3, badgeColor: '#22C55E' },
      ],
    },
    {
      id: 'family-personal',
      title: 'FAMILY & PERSONAL',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'family', label: 'Family', icon: 'Heart' },
        { id: 'favorites', label: 'Favorites', icon: 'Star' },
        { id: 'all-contacts', label: 'All Contacts', icon: 'Users', badge: 42, badgeColor: '#EF4444' },
      ],
    },
  ],
};

// Panel 4: Wallet
export const WALLET: SidebarPanel = {
  id: 'wallet',
  title: 'Wallet',
  icon: 'Wallet',
  badge: 1,
  badgeColor: '#22C55E',
  workspaces: ['all', 'business', 'life'],
  sections: [
    {
      id: 'banking',
      title: 'BANKING',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'bank-accounts', label: 'Bank Accounts', icon: 'Landmark', route: '/billing' },
        { id: 'cash-flow', label: 'Cash Flow', icon: 'TrendingUp', route: '/billing' },
      ],
    },
    {
      id: 'credit-cards',
      title: 'CREDIT CARDS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'credit-cards', label: 'Credit Cards', icon: 'CreditCard', badge: 2, badgeColor: '#22C55E', route: '/billing' },
        { id: 'upcoming-payments', label: 'Upcoming Payments', icon: 'Calendar', badge: 3, badgeColor: '#F59E0B', route: '/billing' },
      ],
    },
    {
      id: 'subscriptions',
      title: 'SUBSCRIPTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'active-subscriptions', label: 'Active Subscriptions', icon: 'Repeat', badge: 12, badgeColor: '#EF4444', route: '/billing' },
        { id: 'upcoming-renewals', label: 'Upcoming Renewals', icon: 'AlertTriangle', badge: 2, badgeColor: '#F59E0B', route: '/billing' },
      ],
    },
  ],
};

// Panel 5: Legal & Compliance
export const LEGAL_COMPLIANCE: SidebarPanel = {
  id: 'legal-compliance',
  title: 'Legal & Compliance',
  icon: 'Scale',
  badge: 1,
  badgeColor: '#22C55E',
  workspaces: ['all', 'business', 'productivity'],
  sections: [
    {
      id: 'contracts-agreements',
      title: 'CONTRACTS & AGREEMENTS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'active-contracts', label: 'Active Contracts', icon: 'FileText', badge: 8, badgeColor: '#22C55E' },
        { id: 'pending-review', label: 'Pending Review', icon: 'AlertTriangle', badge: 3, badgeColor: '#F59E0B' },
        { id: 'signed-documents', label: 'Signed Documents', icon: 'CheckCircle', badge: 24, badgeColor: '#EF4444' },
      ],
    },
    {
      id: 'secrets-keys',
      title: 'SECRETS & KEYS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'api-keys', label: 'API Keys & Credentials', icon: 'Key', badge: 15, badgeColor: '#22C55E' },
        { id: 'password-vault', label: 'Password Vault', icon: 'Lock', badge: 48, badgeColor: '#EF4444' },
        { id: 'certificates', label: 'Certificates & Keys', icon: 'Shield' },
      ],
    },
  ],
};

// Panel 6: Tasks
export const TASKS: SidebarPanel = {
  id: 'tasks',
  title: 'Tasks',
  icon: 'CheckSquare',
  badge: 12,
  badgeColor: '#EF4444',
  workspaces: ['all', 'productivity', 'life', 'business'],
  sections: [
    {
      id: 'my-tasks',
      title: 'MY TASKS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'due-today', label: 'Due today', icon: 'Clock', badge: 3, badgeColor: '#F59E0B' },
        { id: 'in-progress', label: 'In progress', icon: 'Loader', badge: 5, badgeColor: '#EF4444' },
        { id: 'completed', label: 'Completed', icon: 'CheckCircle', badge: 8, badgeColor: '#22C55E' },
      ],
    },
    {
      id: 'other',
      title: 'OTHER',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'priority-tasks', label: 'Priority tasks', icon: 'Flag', badge: 2, badgeColor: '#EF4444' },
        { id: 'archived', label: 'Archived', icon: 'Archive' },
      ],
    },
  ],
};

// Panel 7: Projects
export const PROJECTS: SidebarPanel = {
  id: 'projects',
  title: 'Projects',
  icon: 'FolderKanban',
  workspaces: ['all', 'productivity', 'business'],
  sections: [
    {
      id: 'active-projects',
      title: 'ACTIVE PROJECTS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'ripple-buildkit', label: 'Ripple BuildKit', icon: 'Folder', badge: 5, badgeColor: '#3B82F6' },
        { id: 'holochain-integration', label: 'Holochain Integration', icon: 'Folder', badge: 3, badgeColor: '#22C55E' },
        { id: 'nixos-config', label: 'NixOS Configuration', icon: 'Folder' },
      ],
    },
    {
      id: 'recent',
      title: 'RECENT',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'ui-components', label: 'UI Components', icon: 'Layout' },
        { id: 'api-design', label: 'API Design', icon: 'Code' },
      ],
    },
  ],
};

// Panel 8: Calendar
export const CALENDAR: SidebarPanel = {
  id: 'calendar',
  title: 'Calendar',
  icon: 'Calendar',
  workspaces: ['all', 'productivity', 'life', 'communication'],
  sections: [
    {
      id: 'today',
      title: 'TODAY',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'upcoming-events', label: 'Upcoming Events', icon: 'CalendarClock', badge: 4, badgeColor: '#3B82F6' },
        { id: 'all-day-events', label: 'All Day Events', icon: 'Sun' },
      ],
    },
    {
      id: 'calendars',
      title: 'CALENDARS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'work-calendar', label: 'Work', icon: 'Briefcase', badge: 8, badgeColor: '#22C55E' },
        { id: 'personal-calendar', label: 'Personal', icon: 'User', badge: 3, badgeColor: '#8B5CF6' },
        { id: 'family-calendar', label: 'Family', icon: 'Heart', badge: 2, badgeColor: '#EF4444' },
      ],
    },
  ],
};

// Panel 9: Files
export const FILES: SidebarPanel = {
  id: 'files',
  title: 'Files',
  icon: 'Folder',
  workspaces: ['all', 'productivity', 'business'],
  sections: [
    {
      id: 'quick-access',
      title: 'QUICK ACCESS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'recent-files', label: 'Recent', icon: 'Clock', route: '/files' },
        { id: 'starred-files', label: 'Starred', icon: 'Star', badge: 5, badgeColor: '#F59E0B', route: '/files' },
        { id: 'shared-files', label: 'Shared with me', icon: 'Share2', badge: 12, badgeColor: '#3B82F6', route: '/files' },
      ],
    },
    {
      id: 'locations',
      title: 'LOCATIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'my-drive', label: 'My Drive', icon: 'HardDrive', route: '/files' },
        { id: 'cloud-storage', label: 'Cloud Storage', icon: 'Cloud', route: '/files' },
        { id: 'local-files', label: 'Local Files', icon: 'Laptop', route: '/files' },
      ],
    },
  ],
};

// Panel 10: Analytics Hub
export const ANALYTICS_HUB: SidebarPanel = {
  id: 'analytics-hub',
  title: 'Analytics Hub',
  icon: 'BarChart2',
  workspaces: ['all', 'productivity', 'business', 'infrastructure'],
  sections: [
    {
      id: 'dashboards',
      title: 'DASHBOARDS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'overview-dashboard', label: 'Overview', icon: 'LayoutDashboard', route: '/dashboard' },
        { id: 'performance-dashboard', label: 'Performance', icon: 'TrendingUp', badge: 3, badgeColor: '#22C55E', route: '/dashboard' },
        { id: 'usage-dashboard', label: 'Usage Analytics', icon: 'Activity', route: '/dashboard' },
      ],
    },
    {
      id: 'reports',
      title: 'REPORTS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'weekly-report', label: 'Weekly Report', icon: 'FileText' },
        { id: 'monthly-report', label: 'Monthly Report', icon: 'FileText' },
        { id: 'custom-reports', label: 'Custom Reports', icon: 'Sliders' },
      ],
    },
  ],
};

// Panel 11: Social Media & Apps
export const SOCIAL_MEDIA: SidebarPanel = {
  id: 'social-media',
  title: 'Social Media & Apps',
  icon: 'Share2',
  badge: 4,
  badgeColor: '#EF4444',
  workspaces: ['all', 'life', 'business'],
  sections: [
    {
      id: 'social-accounts',
      title: 'SOCIAL ACCOUNTS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'twitter', label: 'Twitter/X', icon: 'Twitter', badge: 12, badgeColor: '#3B82F6' },
        { id: 'linkedin', label: 'LinkedIn', icon: 'Linkedin', badge: 5, badgeColor: '#0077B5' },
        { id: 'github', label: 'GitHub', icon: 'Github', badge: 8, badgeColor: '#22C55E' },
      ],
    },
    {
      id: 'connected-apps',
      title: 'CONNECTED APPS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'slack', label: 'Slack', icon: 'MessageSquare', badge: 23, badgeColor: '#EF4444' },
        { id: 'discord', label: 'Discord', icon: 'MessageCircle', badge: 7, badgeColor: '#5865F2' },
        { id: 'notion', label: 'Notion', icon: 'BookOpen' },
      ],
    },
  ],
};

// Panel 12: Compute Nodes
export const COMPUTE_NODES: SidebarPanel = {
  id: 'compute-nodes',
  title: 'Compute Nodes',
  icon: 'Server',
  workspaces: ['all', 'infrastructure', 'productivity'],
  sections: [
    {
      id: 'active-nodes',
      title: 'ACTIVE NODES',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'node-1', label: 'Primary Node', icon: 'Server', status: 'online', badge: 1, badgeColor: '#22C55E' },
        { id: 'node-2', label: 'Secondary Node', icon: 'Server', status: 'online' },
        { id: 'node-3', label: 'Edge Node', icon: 'Server', status: 'busy', badge: 3, badgeColor: '#F59E0B' },
      ],
    },
    {
      id: 'resources',
      title: 'RESOURCES',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'cpu-usage', label: 'CPU Usage', icon: 'Cpu', metadata: '45%' },
        { id: 'memory-usage', label: 'Memory', icon: 'MemoryStick', metadata: '8.2GB / 16GB' },
        { id: 'storage-usage', label: 'Storage', icon: 'HardDrive', metadata: '256GB / 1TB' },
      ],
    },
  ],
};

// Panel 13: Notifications
export const NOTIFICATIONS: SidebarPanel = {
  id: 'notifications',
  title: 'Notifications',
  icon: 'Bell',
  badge: 6,
  badgeColor: '#EF4444',
  workspaces: ['all', 'productivity', 'life', 'communication', 'intelligence', 'infrastructure', 'business'],
  sections: [
    {
      id: 'unread',
      title: 'UNREAD',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'notif-1', label: 'Build completed', icon: 'CheckCircle', metadata: '2 min ago', badgeColor: '#22C55E', route: '/notifications' },
        { id: 'notif-2', label: 'New message from Team', icon: 'MessageCircle', metadata: '15 min ago', badgeColor: '#3B82F6', route: '/notifications' },
        { id: 'notif-3', label: 'Security alert', icon: 'AlertTriangle', metadata: '1 hour ago', badgeColor: '#EF4444', route: '/notifications' },
      ],
    },
    {
      id: 'earlier',
      title: 'EARLIER',
      collapsible: true,
      defaultExpanded: false,
      items: [
        { id: 'notif-4', label: 'Task assigned', icon: 'Clipboard', metadata: 'Yesterday', route: '/notifications' },
        { id: 'notif-5', label: 'Meeting reminder', icon: 'Calendar', metadata: 'Yesterday', route: '/notifications' },
        { id: 'notif-6', label: 'System update', icon: 'Download', metadata: '2 days ago', route: '/notifications' },
      ],
    },
  ],
};

// Panel 14: Favorites
export const FAVORITES: SidebarPanel = {
  id: 'favorites',
  title: 'Favorites',
  icon: 'Star',
  badge: 5,
  badgeColor: '#3B82F6',
  workspaces: ['all', 'productivity', 'life', 'communication', 'intelligence', 'infrastructure', 'business'],
  sections: [
    {
      id: 'pinned',
      title: 'PINNED',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'fav-1', label: 'AI Command Center', icon: 'Layers', route: '/chat' },
        { id: 'fav-2', label: 'Tasks', icon: 'CheckSquare', route: '/dashboard' },
        { id: 'fav-3', label: 'Projects', icon: 'FolderKanban', route: '/dashboard' },
      ],
    },
    {
      id: 'recent-favorites',
      title: 'RECENTLY ADDED',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'fav-4', label: 'Compute Nodes', icon: 'Server', route: '/dashboard' },
        { id: 'fav-5', label: 'Analytics Hub', icon: 'BarChart2', route: '/dashboard' },
      ],
    },
  ],
};

// Panel 15: Settings
export const SETTINGS: SidebarPanel = {
  id: 'settings',
  title: 'Settings',
  icon: 'Settings',
  workspaces: ['all', 'productivity', 'life', 'communication', 'intelligence', 'infrastructure', 'business'],
  sections: [
    {
      id: 'account',
      title: 'ACCOUNT',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'profile-settings', label: 'Profile', icon: 'User', route: '/settings' },
        { id: 'security-settings', label: 'Security', icon: 'Shield', route: '/settings' },
        { id: 'privacy-settings', label: 'Privacy', icon: 'EyeOff', route: '/settings' },
      ],
    },
    {
      id: 'preferences',
      title: 'PREFERENCES',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'appearance', label: 'Appearance', icon: 'Palette', route: '/settings' },
        { id: 'notifications-pref', label: 'Notifications', icon: 'Bell', route: '/settings' },
        { id: 'language', label: 'Language & Region', icon: 'Globe', route: '/settings' },
      ],
    },
    {
      id: 'integrations',
      title: 'INTEGRATIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'connected-services', label: 'Connected Services', icon: 'Link', route: '/settings' },
        { id: 'api-settings', label: 'API Settings', icon: 'Code', route: '/settings' },
      ],
    },
  ],
};

// Panel 16: Profile
export const PROFILE: SidebarPanel = {
  id: 'profile',
  title: 'Profile',
  icon: 'User',
  workspaces: ['all', 'productivity', 'life', 'communication', 'intelligence', 'infrastructure', 'business'],
  sections: [
    {
      id: 'my-profile',
      title: 'MY PROFILE',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'view-profile', label: 'View Profile', icon: 'User', route: '/settings' },
        { id: 'edit-profile', label: 'Edit Profile', icon: 'Edit', route: '/settings' },
        { id: 'activity-status', label: 'Activity Status', icon: 'Activity', status: 'online' },
      ],
    },
    {
      id: 'account-actions',
      title: 'ACCOUNT',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'switch-account', label: 'Switch Account', icon: 'Users' },
        { id: 'sign-out', label: 'Sign Out', icon: 'LogOut' },
      ],
    },
  ],
};

// All panels in order for "All" workspace
export const ALL_PANELS: SidebarPanel[] = [
  AI_COMMAND_CENTER,
  LIFE_DASHBOARD,
  CONTACTS,
  WALLET,
  LEGAL_COMPLIANCE,
  TASKS,
  PROJECTS,
  CALENDAR,
  FILES,
  ANALYTICS_HUB,
  SOCIAL_MEDIA,
  COMPUTE_NODES,
  NOTIFICATIONS,
  FAVORITES,
  SETTINGS,
  PROFILE,
];

// Get panels for a specific workspace
export function getPanelsForWorkspace(workspace: WorkspaceType): SidebarPanel[] {
  if (workspace === 'all') {
    return ALL_PANELS;
  }
  return ALL_PANELS.filter(panel => panel.workspaces.includes(workspace));
}

// Get a specific panel by ID
export function getPanelById(id: string): SidebarPanel | undefined {
  return ALL_PANELS.find(panel => panel.id === id);
}

// Icon rail items derived from panels
export interface IconRailItem {
  id: string;
  icon: string;
  label: string;
  badge?: number;
  badgeColor?: string;
  panelId: string;
}

export function getIconRailItems(workspace: WorkspaceType): IconRailItem[] {
  const panels = getPanelsForWorkspace(workspace);
  return panels.map(panel => ({
    id: panel.id,
    icon: panel.icon,
    label: panel.title,
    badge: panel.badge,
    badgeColor: panel.badgeColor,
    panelId: panel.id,
  }));
}


// Reverse mapping: route -> section for auto-selection
export const ROUTE_TO_SECTION: Record<string, string> = {
  '/dashboard': 'life-dashboard',
  '/chat': 'ai-command-center',
  '/files': 'files',
  '/notifications': 'notifications',
  '/billing': 'wallet',
  '/settings': 'settings',
};
