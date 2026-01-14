// Complete sidebar panel data matching Figma design screenshots
// Each panel corresponds to an icon in the icon rail

export type WorkspaceType = 'all' | 'productivity' | 'communication' | 'intelligence' | 'infrastructure' | 'life' | 'business';

export interface WorkspaceOption {
  id: WorkspaceType;
  label: string;
  icon: string;
}

export const WORKSPACES: WorkspaceOption[] = [
  { id: 'all', label: 'All', icon: 'layers' },
  { id: 'productivity', label: 'Work', icon: 'briefcase' },
  { id: 'communication', label: 'People', icon: 'users' },
  { id: 'intelligence', label: 'AI', icon: 'bot' },
  { id: 'infrastructure', label: 'Tech', icon: 'cpu' },
  { id: 'life', label: 'Life', icon: 'heart' },
  { id: 'business', label: 'Business', icon: 'building' },
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
  workspaces: WorkspaceType[]; // Which workspaces show this panel
}

// Panel 1: AI Command Center
export const AI_COMMAND_CENTER: SidebarPanel = {
  id: 'ai-command-center',
  title: 'AI Command Center',
  icon: 'layers',
  badge: 3,
  badgeColor: '#22C55E', // green
  workspaces: ['all', 'intelligence', 'productivity', 'infrastructure'],
  sections: [
    {
      id: 'active-conversations',
      title: 'ACTIVE CONVERSATIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'life-assistant', label: 'Life Assistant', icon: 'message-square', metadata: 'Running your day', badge: 2, badgeColor: '#EF4444' },
        { id: 'work-strategy', label: 'Work Strategy', icon: 'message-square', metadata: 'Project planning' },
        { id: 'home-automation', label: 'Home Automation', icon: 'message-square', metadata: 'Managing smart home' },
      ],
    },
    {
      id: 'ai-knowledge-base',
      title: 'AI KNOWLEDGE BASE',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'goals-objectives', label: 'Goals & Objectives', icon: 'target', badge: 8, badgeColor: '#EF4444' },
        { id: 'ideas-insights', label: 'Ideas & Insights', icon: 'lightbulb', badge: 12, badgeColor: '#3B82F6' },
        { id: 'rules-protocols', label: 'Rules & Protocols', icon: 'shield' },
        { id: 'memory-context', label: 'Memory & Context', icon: 'brain' },
      ],
    },
    {
      id: 'ai-capabilities',
      title: 'AI CAPABILITIES',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'multi-modal', label: 'Multi-Modal Interface', icon: 'layout-grid' },
        { id: 'specialized-assistants', label: 'Specialized Assistants', icon: 'bot' },
        { id: 'automation-workflows', label: 'Automation & Workflows', icon: 'workflow', badge: 15, badgeColor: '#3B82F6' },
      ],
    },
  ],
};

// Panel 2: Life Dashboard
export const LIFE_DASHBOARD: SidebarPanel = {
  id: 'life-dashboard',
  title: 'Life Dashboard',
  icon: 'layout-dashboard',
  workspaces: ['all', 'life', 'communication'],
  sections: [
    {
      id: 'overview',
      title: 'OVERVIEW',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'todays-summary', label: "Today's Summary", icon: 'calendar-check', metadata: 'All areas' },
        { id: 'key-metrics', label: 'Key Metrics', icon: 'trending-up' },
      ],
    },
    {
      id: 'domain-dashboards',
      title: 'DOMAIN DASHBOARDS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'work-productivity', label: 'Work & Productivity', icon: 'briefcase', badge: 5, badgeColor: '#22C55E' },
        { id: 'family-personal', label: 'Family & Personal', icon: 'heart', badge: 2, badgeColor: '#3B82F6' },
        { id: 'home-environment', label: 'Home & Environment', icon: 'home' },
        { id: 'analytics-hub', label: 'Analytics Hub', icon: 'bar-chart-2' },
      ],
    },
    {
      id: 'recent-activity',
      title: 'RECENT ACTIVITY',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'activity-feed', label: 'Activity Feed', icon: 'activity', badge: 8, badgeColor: '#EF4444' },
      ],
    },
  ],
};

// Panel 3: Contacts
export const CONTACTS: SidebarPanel = {
  id: 'contacts',
  title: 'Contacts',
  icon: 'users',
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
        { id: 'video-conferencing', label: 'Video Conferencing', icon: 'video', badge: 1, badgeColor: '#F59E0B' },
        { id: 'calls', label: 'Calls', icon: 'phone', badge: 2, badgeColor: '#EF4444' },
        { id: 'messaging', label: 'Messaging', icon: 'message-circle', badge: 5, badgeColor: '#22C55E' },
      ],
    },
    {
      id: 'work-contacts',
      title: 'WORK CONTACTS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'dev-team', label: 'Development Team', icon: 'users', badge: 4, badgeColor: '#3B82F6' },
        { id: 'design-team', label: 'Design Team', icon: 'users', badge: 3, badgeColor: '#22C55E' },
      ],
    },
    {
      id: 'family-personal',
      title: 'FAMILY & PERSONAL',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'family', label: 'Family', icon: 'heart' },
        { id: 'favorites', label: 'Favorites', icon: 'star' },
        { id: 'all-contacts', label: 'All Contacts', icon: 'users', badge: 42, badgeColor: '#EF4444' },
      ],
    },
    {
      id: 'location-presence',
      title: 'LOCATION & PRESENCE',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'people-nearby', label: 'People Nearby', icon: 'map-pin' },
      ],
    },
  ],
};

// Panel 4: Wallet
export const WALLET: SidebarPanel = {
  id: 'wallet',
  title: 'Wallet',
  icon: 'wallet',
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
        { id: 'bank-accounts', label: 'Bank Accounts', icon: 'landmark' },
        { id: 'cash-flow', label: 'Cash Flow', icon: 'trending-up' },
      ],
    },
    {
      id: 'credit-cards',
      title: 'CREDIT CARDS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'credit-cards', label: 'Credit Cards', icon: 'credit-card', badge: 2, badgeColor: '#22C55E' },
        { id: 'upcoming-payments', label: 'Upcoming Payments', icon: 'calendar', badge: 3, badgeColor: '#F59E0B' },
      ],
    },
    {
      id: 'subscriptions',
      title: 'SUBSCRIPTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'active-subscriptions', label: 'Active Subscriptions', icon: 'repeat', badge: 12, badgeColor: '#EF4444' },
        { id: 'upcoming-renewals', label: 'Upcoming Renewals', icon: 'alert-triangle', badge: 2, badgeColor: '#F59E0B' },
      ],
    },
    {
      id: 'investments',
      title: 'INVESTMENTS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'portfolio-overview', label: 'Portfolio Overview', icon: 'pie-chart' },
        { id: 'performance', label: 'Performance', icon: 'bar-chart' },
      ],
    },
    {
      id: 'budgeting',
      title: 'BUDGETING',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'budget-overview', label: 'Budget Overview', icon: 'calculator' },
        { id: 'spending-analysis', label: 'Spending Analysis', icon: 'pie-chart' },
      ],
    },
  ],
};

// Panel 5: Legal & Compliance
export const LEGAL_COMPLIANCE: SidebarPanel = {
  id: 'legal-compliance',
  title: 'Legal & Compliance',
  icon: 'scale',
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
        { id: 'active-contracts', label: 'Active Contracts', icon: 'file-text', badge: 8, badgeColor: '#22C55E' },
        { id: 'pending-review', label: 'Pending Review', icon: 'alert-triangle', badge: 3, badgeColor: '#F59E0B' },
        { id: 'signed-documents', label: 'Signed Documents', icon: 'check-circle', badge: 24, badgeColor: '#EF4444' },
      ],
    },
    {
      id: 'secrets-keys',
      title: 'SECRETS & KEYS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'api-keys', label: 'API Keys & Credentials', icon: 'key', badge: 15, badgeColor: '#22C55E' },
        { id: 'password-vault', label: 'Password Vault', icon: 'lock', badge: 48, badgeColor: '#EF4444' },
        { id: 'certificates', label: 'Certificates & Keys', icon: 'shield' },
      ],
    },
    {
      id: 'governance-compliance',
      title: 'GOVERNANCE & COMPLIANCE',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'compliance-status', label: 'Compliance Status', icon: 'check-square' },
        { id: 'policies-procedures', label: 'Policies & Procedures', icon: 'book' },
        { id: 'audits-assessments', label: 'Audits & Assessments', icon: 'clipboard-list' },
      ],
    },
    {
      id: 'quick-actions',
      title: 'QUICK ACTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'new-contract', label: 'New Contract', icon: 'plus' },
        { id: 'request-signature', label: 'Request Signature', icon: 'pen-tool' },
      ],
    },
  ],
};

// Panel 6: Tasks
export const TASKS: SidebarPanel = {
  id: 'tasks',
  title: 'Tasks',
  icon: 'check-square',
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
        { id: 'due-today', label: 'Due today', icon: 'clock', badge: 3, badgeColor: '#F59E0B' },
        { id: 'in-progress', label: 'In progress', icon: 'loader', badge: 5, badgeColor: '#EF4444' },
        { id: 'completed', label: 'Completed', icon: 'check-circle', badge: 8, badgeColor: '#22C55E' },
      ],
    },
    {
      id: 'other',
      title: 'OTHER',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'priority-tasks', label: 'Priority tasks', icon: 'flag', badge: 2, badgeColor: '#EF4444' },
        { id: 'archived', label: 'Archived', icon: 'archive' },
      ],
    },
    {
      id: 'quick-actions',
      title: 'QUICK ACTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'new-task', label: 'New task', icon: 'plus' },
        { id: 'filter-tasks', label: 'Filter tasks', icon: 'filter' },
      ],
    },
  ],
};

// Panel 7: Projects
export const PROJECTS: SidebarPanel = {
  id: 'projects',
  title: 'Projects',
  icon: 'folder-kanban',
  workspaces: ['all', 'productivity', 'business'],
  sections: [
    {
      id: 'active-projects',
      title: 'ACTIVE PROJECTS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'ripple-buildkit', label: 'Ripple BuildKit', icon: 'folder', badge: 5, badgeColor: '#3B82F6' },
        { id: 'holochain-integration', label: 'Holochain Integration', icon: 'folder', badge: 3, badgeColor: '#22C55E' },
        { id: 'nixos-config', label: 'NixOS Configuration', icon: 'folder' },
      ],
    },
    {
      id: 'recent',
      title: 'RECENT',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'ui-components', label: 'UI Components', icon: 'layout' },
        { id: 'api-design', label: 'API Design', icon: 'code' },
      ],
    },
    {
      id: 'quick-actions',
      title: 'QUICK ACTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'new-project', label: 'New Project', icon: 'plus' },
        { id: 'import-project', label: 'Import Project', icon: 'download' },
      ],
    },
  ],
};

// Panel 8: Calendar
export const CALENDAR: SidebarPanel = {
  id: 'calendar',
  title: 'Calendar',
  icon: 'calendar',
  workspaces: ['all', 'productivity', 'life', 'communication'],
  sections: [
    {
      id: 'today',
      title: 'TODAY',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'upcoming-events', label: 'Upcoming Events', icon: 'calendar-clock', badge: 4, badgeColor: '#3B82F6' },
        { id: 'all-day-events', label: 'All Day Events', icon: 'sun' },
      ],
    },
    {
      id: 'calendars',
      title: 'CALENDARS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'work-calendar', label: 'Work', icon: 'briefcase', badge: 8, badgeColor: '#22C55E' },
        { id: 'personal-calendar', label: 'Personal', icon: 'user', badge: 3, badgeColor: '#8B5CF6' },
        { id: 'family-calendar', label: 'Family', icon: 'heart', badge: 2, badgeColor: '#EF4444' },
      ],
    },
    {
      id: 'quick-actions',
      title: 'QUICK ACTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'new-event', label: 'New Event', icon: 'plus' },
        { id: 'schedule-meeting', label: 'Schedule Meeting', icon: 'users' },
      ],
    },
  ],
};

// Panel 9: Files
export const FILES: SidebarPanel = {
  id: 'files',
  title: 'Files',
  icon: 'folder',
  workspaces: ['all', 'productivity', 'business'],
  sections: [
    {
      id: 'quick-access',
      title: 'QUICK ACCESS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'recent-files', label: 'Recent', icon: 'clock' },
        { id: 'starred-files', label: 'Starred', icon: 'star', badge: 5, badgeColor: '#F59E0B' },
        { id: 'shared-files', label: 'Shared with me', icon: 'share-2', badge: 12, badgeColor: '#3B82F6' },
      ],
    },
    {
      id: 'locations',
      title: 'LOCATIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'my-drive', label: 'My Drive', icon: 'hard-drive' },
        { id: 'cloud-storage', label: 'Cloud Storage', icon: 'cloud' },
        { id: 'local-files', label: 'Local Files', icon: 'laptop' },
      ],
    },
    {
      id: 'quick-actions',
      title: 'QUICK ACTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'upload-file', label: 'Upload File', icon: 'upload' },
        { id: 'new-folder', label: 'New Folder', icon: 'folder-plus' },
      ],
    },
  ],
};

// Panel 10: Analytics Hub
export const ANALYTICS_HUB: SidebarPanel = {
  id: 'analytics-hub',
  title: 'Analytics Hub',
  icon: 'bar-chart-2',
  workspaces: ['all', 'productivity', 'business', 'infrastructure'],
  sections: [
    {
      id: 'dashboards',
      title: 'DASHBOARDS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'overview-dashboard', label: 'Overview', icon: 'layout-dashboard' },
        { id: 'performance-dashboard', label: 'Performance', icon: 'trending-up', badge: 3, badgeColor: '#22C55E' },
        { id: 'usage-dashboard', label: 'Usage Analytics', icon: 'activity' },
      ],
    },
    {
      id: 'reports',
      title: 'REPORTS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'weekly-report', label: 'Weekly Report', icon: 'file-text' },
        { id: 'monthly-report', label: 'Monthly Report', icon: 'file-text' },
        { id: 'custom-reports', label: 'Custom Reports', icon: 'sliders' },
      ],
    },
    {
      id: 'quick-actions',
      title: 'QUICK ACTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'create-report', label: 'Create Report', icon: 'plus' },
        { id: 'export-data', label: 'Export Data', icon: 'download' },
      ],
    },
  ],
};

// Panel 11: Social Media & Apps
export const SOCIAL_MEDIA: SidebarPanel = {
  id: 'social-media',
  title: 'Social Media & Apps',
  icon: 'share-2',
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
        { id: 'twitter', label: 'Twitter/X', icon: 'twitter', badge: 12, badgeColor: '#3B82F6' },
        { id: 'linkedin', label: 'LinkedIn', icon: 'linkedin', badge: 5, badgeColor: '#0077B5' },
        { id: 'github', label: 'GitHub', icon: 'github', badge: 8, badgeColor: '#22C55E' },
      ],
    },
    {
      id: 'connected-apps',
      title: 'CONNECTED APPS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'slack', label: 'Slack', icon: 'message-square', badge: 23, badgeColor: '#EF4444' },
        { id: 'discord', label: 'Discord', icon: 'message-circle', badge: 7, badgeColor: '#5865F2' },
        { id: 'notion', label: 'Notion', icon: 'book-open' },
      ],
    },
    {
      id: 'quick-actions',
      title: 'QUICK ACTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'connect-app', label: 'Connect App', icon: 'plus' },
        { id: 'manage-accounts', label: 'Manage Accounts', icon: 'settings' },
      ],
    },
  ],
};

// Panel 12: Compute Nodes
export const COMPUTE_NODES: SidebarPanel = {
  id: 'compute-nodes',
  title: 'Compute Nodes',
  icon: 'server',
  workspaces: ['all', 'infrastructure', 'productivity'],
  sections: [
    {
      id: 'active-nodes',
      title: 'ACTIVE NODES',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'node-1', label: 'Primary Node', icon: 'server', status: 'online', badge: 1, badgeColor: '#22C55E' },
        { id: 'node-2', label: 'Secondary Node', icon: 'server', status: 'online' },
        { id: 'node-3', label: 'Edge Node', icon: 'server', status: 'busy', badge: 3, badgeColor: '#F59E0B' },
      ],
    },
    {
      id: 'resources',
      title: 'RESOURCES',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'cpu-usage', label: 'CPU Usage', icon: 'cpu', metadata: '45%' },
        { id: 'memory-usage', label: 'Memory', icon: 'memory-stick', metadata: '8.2GB / 16GB' },
        { id: 'storage-usage', label: 'Storage', icon: 'hard-drive', metadata: '256GB / 1TB' },
      ],
    },
    {
      id: 'quick-actions',
      title: 'QUICK ACTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'add-node', label: 'Add Node', icon: 'plus' },
        { id: 'scale-resources', label: 'Scale Resources', icon: 'maximize-2' },
      ],
    },
  ],
};

// Panel 13: Notifications (already partially implemented)
export const NOTIFICATIONS: SidebarPanel = {
  id: 'notifications',
  title: 'Notifications',
  icon: 'bell',
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
        { id: 'notif-1', label: 'Build completed', icon: 'check-circle', metadata: '2 min ago', badgeColor: '#22C55E' },
        { id: 'notif-2', label: 'New message from Team', icon: 'message-circle', metadata: '15 min ago', badgeColor: '#3B82F6' },
        { id: 'notif-3', label: 'Security alert', icon: 'alert-triangle', metadata: '1 hour ago', badgeColor: '#EF4444' },
      ],
    },
    {
      id: 'earlier',
      title: 'EARLIER',
      collapsible: true,
      defaultExpanded: false,
      items: [
        { id: 'notif-4', label: 'Task assigned', icon: 'clipboard', metadata: 'Yesterday' },
        { id: 'notif-5', label: 'Meeting reminder', icon: 'calendar', metadata: 'Yesterday' },
        { id: 'notif-6', label: 'System update', icon: 'download', metadata: '2 days ago' },
      ],
    },
    {
      id: 'quick-actions',
      title: 'QUICK ACTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'mark-all-read', label: 'Mark all as read', icon: 'check' },
        { id: 'notification-settings', label: 'Notification Settings', icon: 'settings' },
      ],
    },
  ],
};

// Panel 14: Favorites
export const FAVORITES: SidebarPanel = {
  id: 'favorites',
  title: 'Favorites',
  icon: 'star',
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
        { id: 'fav-1', label: 'AI Command Center', icon: 'layers' },
        { id: 'fav-2', label: 'Tasks', icon: 'check-square' },
        { id: 'fav-3', label: 'Projects', icon: 'folder-kanban' },
      ],
    },
    {
      id: 'recent-favorites',
      title: 'RECENTLY ADDED',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'fav-4', label: 'Compute Nodes', icon: 'server' },
        { id: 'fav-5', label: 'Analytics Hub', icon: 'bar-chart-2' },
      ],
    },
    {
      id: 'quick-actions',
      title: 'QUICK ACTIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'manage-favorites', label: 'Manage Favorites', icon: 'settings' },
      ],
    },
  ],
};

// Panel 15: Settings
export const SETTINGS: SidebarPanel = {
  id: 'settings',
  title: 'Settings',
  icon: 'settings',
  workspaces: ['all', 'productivity', 'life', 'communication', 'intelligence', 'infrastructure', 'business'],
  sections: [
    {
      id: 'account',
      title: 'ACCOUNT',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'profile-settings', label: 'Profile', icon: 'user' },
        { id: 'security-settings', label: 'Security', icon: 'shield' },
        { id: 'privacy-settings', label: 'Privacy', icon: 'eye-off' },
      ],
    },
    {
      id: 'preferences',
      title: 'PREFERENCES',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'appearance', label: 'Appearance', icon: 'palette' },
        { id: 'notifications-pref', label: 'Notifications', icon: 'bell' },
        { id: 'language', label: 'Language & Region', icon: 'globe' },
      ],
    },
    {
      id: 'integrations',
      title: 'INTEGRATIONS',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'connected-services', label: 'Connected Services', icon: 'link' },
        { id: 'api-settings', label: 'API Settings', icon: 'code' },
      ],
    },
  ],
};

// Panel 16: Profile
export const PROFILE: SidebarPanel = {
  id: 'profile',
  title: 'Profile',
  icon: 'user',
  workspaces: ['all', 'productivity', 'life', 'communication', 'intelligence', 'infrastructure', 'business'],
  sections: [
    {
      id: 'my-profile',
      title: 'MY PROFILE',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'view-profile', label: 'View Profile', icon: 'user' },
        { id: 'edit-profile', label: 'Edit Profile', icon: 'edit' },
        { id: 'activity-status', label: 'Activity Status', icon: 'activity', status: 'online' },
      ],
    },
    {
      id: 'account-actions',
      title: 'ACCOUNT',
      collapsible: true,
      defaultExpanded: true,
      items: [
        { id: 'switch-account', label: 'Switch Account', icon: 'users' },
        { id: 'sign-out', label: 'Sign Out', icon: 'log-out' },
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
