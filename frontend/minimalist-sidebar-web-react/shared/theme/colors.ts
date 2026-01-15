/**
 * Shared Color Theme for Minimalist Sidebar
 * Source of truth for both React and Vue variants
 */

export const colors = {
  // Brand colors
  brand: {
    primary: '#030213',
    secondary: '#717182',
    accent: '#e9ebef',
  },

  // Status colors
  status: {
    online: '#22c55e',
    offline: '#6b7280',
    away: '#f59e0b',
    busy: '#ef4444',
    good: '#22c55e',
    warning: '#f59e0b',
    error: '#ef4444',
    info: '#3b82f6',
  },

  // Badge colors
  badge: {
    info: { bg: '#dbeafe', text: '#1e40af' },
    success: { bg: '#dcfce7', text: '#166534' },
    warning: { bg: '#fef3c7', text: '#92400e' },
    error: { bg: '#fee2e2', text: '#991b1b' },
  },

  // Sidebar specific
  sidebar: {
    light: {
      background: '#ffffff',
      foreground: '#030213',
      border: 'rgba(0, 0, 0, 0.1)',
      hover: '#f3f3f5',
      active: '#e9ebef',
    },
    dark: {
      background: '#1a1a1a',
      foreground: '#ffffff',
      border: 'rgba(255, 255, 255, 0.1)',
      hover: '#2a2a2a',
      active: '#3a3a3a',
    },
  },
};

export const iconRailWidth = 64;
export const detailSidebarWidth = 280;
export const collapsedWidth = 64;

export default colors;
