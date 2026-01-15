/**
 * Ripple UI Component Library
 * 
 * Export all UI components for easy importing throughout the application.
 * Usage: import { Button, Card, Badge } from '@/components/ui';
 */

// Core Radix-based components
export * from './accordion';
export * from './alert-dialog';
export * from './avatar';
export * from './badge';
export * from './button';
export * from './card';
export * from './checkbox';
export * from './collapsible';
export * from './dialog';
export * from './dropdown-menu';
export * from './input';
export * from './label';
export * from './popover';
export * from './progress';
export * from './radio-group';
export * from './scroll-area';
export * from './select';
export * from './separator';
export * from './sheet';
export * from './skeleton';
export * from './slider';
export * from './switch';
export * from './tabs';
export * from './textarea';
export * from './toast';
export * from './toaster';
export * from './tooltip';
export * from './use-toast';

// Re-export Ripple brand colors
export const RIPPLE_COLORS = {
  primary: '#00D4FF',
  secondary: '#9B7BFF',
  accent: '#00E676',
  warning: '#FFB300',
  error: '#FF5252',
  foreground: '#ECEDEE',
  muted: '#9BA1A6',
  border: '#2A2A2A',
  surface: '#1A1A1A',
  background: '#0A0A0A',
} as const;

// Gradient presets
export const RIPPLE_GRADIENTS = {
  primary: 'linear-gradient(135deg, #00D4FF 0%, #9B7BFF 100%)',
  accent: 'linear-gradient(135deg, #9B7BFF 0%, #00E676 100%)',
  full: 'linear-gradient(135deg, #00D4FF 0%, #9B7BFF 50%, #00E676 100%)',
} as const;
