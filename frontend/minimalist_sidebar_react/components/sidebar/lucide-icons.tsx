import React from 'react';
import { View, Text, StyleSheet } from 'react-native';
import {
  Bot,
  LayoutDashboard,
  Users,
  Wallet,
  Scale,
  Hammer,
  BrainCircuit,
  CheckSquare,
  Folder,
  Calendar,
  FileText,
  BarChart3,
  Share2,
  Network,
  Home,
  ShieldAlert,
  Settings,
  MessageSquare,
  Target,
  Lightbulb,
  Shield,
  Brain,
  Sparkles,
  Workflow,
  Play,
  ScrollText,
  Rocket,
  Crown,
  Search,
  ShieldCheck,
  GitBranch,
  Server,
  Cpu,
  HardDrive,
  Database,
  Plug,
  Activity,
  Timer,
  User,
  Palette,
  Bell,
  Wifi,
  Info,
  RefreshCw,
  HelpCircle,
  Box,
  Terminal,
  Package,
  TestTube,
  GitMerge,
  ListTodo,
  ListOrdered,
  BookOpen,
  CheckCircle,
  XCircle,
  Sun,
  Moon,
  Zap,
  DollarSign,
  Heart,
  Lock,
  Eye,
  EyeOff,
  Mic,
  Languages,
  Briefcase,
  Plane,
  Edit,
  Image,
  Star,
  ChevronDown,
  ChevronRight,
  ChevronLeft,
  ChevronUp,
  X,
  Plus,
  Clock,
  AlertTriangle,
  TrendingUp,
  ArrowUp,
  ArrowDown,
  Minus,
  Key,
  Book,
  Link,
  RotateCcw,
  Loader,
  Globe,
  Sliders,
  Type,
  PanelLeft,
  MemoryStick,
  Layers,
  MessageCircle,
  Video,
  Phone,
  CreditCard,
  Building,
  Landmark,
  FolderKanban,
  CalendarCheck,
  CalendarClock,
  PieChart,
  BarChart2,
  TrendingDown,
  Mail,
  Send,
  UserPlus,
  Clipboard,
  ClipboardList,
  Flag,
  Archive,
  Filter,
  Download,
  Upload,
  Maximize2,
  PenTool,
  Calculator,
  AlertCircle,
  Repeat,
  Twitter,
  Linkedin,
  Github,
  MapPin,
  Cloud,
  Laptop,
  LogOut,
  FolderPlus,
  LayoutGrid,
  Building2,
} from 'lucide-react-native';

// Icon mapping from string names to components
const iconMap: Record<string, React.ComponentType<{ size?: number; color?: string; className?: string }>> = {
  // Navigation & Layout
  'layers': Layers,
  'Layers': Layers,
  'layout-dashboard': LayoutDashboard,
  'LayoutDashboard': LayoutDashboard,
  'dashboard': LayoutDashboard,
  'layout-grid': LayoutGrid,
  'LayoutGrid': LayoutGrid,
  'layout': LayoutDashboard,
  'menu': PanelLeft,
  'panel-left': PanelLeft,
  'PanelLeft': PanelLeft,
  'chevron-down': ChevronDown,
  'ChevronDown': ChevronDown,
  'chevron-right': ChevronRight,
  'ChevronRight': ChevronRight,
  'chevron-left': ChevronLeft,
  'ChevronLeft': ChevronLeft,
  'chevron-up': ChevronUp,
  'ChevronUp': ChevronUp,
  'x': X,
  'X': X,
  
  // Communication
  'message-square': MessageSquare,
  'MessageSquare': MessageSquare,
  'message-circle': MessageCircle,
  'MessageCircle': MessageCircle,
  'video': Video,
  'Video': Video,
  'phone': Phone,
  'Phone': Phone,
  'mail': Mail,
  'Mail': Mail,
  'send': Send,
  'Send': Send,
  
  // Users & People
  'user': User,
  'User': User,
  'users': Users,
  'Users': Users,
  'user-plus': UserPlus,
  'UserPlus': UserPlus,
  
  // Business & Work
  'briefcase': Briefcase,
  'Briefcase': Briefcase,
  'building': Building,
  'Building': Building,
  'building-2': Building2,
  'Building2': Building2,
  'landmark': Landmark,
  'Landmark': Landmark,
  'wallet': Wallet,
  'Wallet': Wallet,
  'credit-card': CreditCard,
  'CreditCard': CreditCard,
  'scale': Scale,
  'Scale': Scale,
  'balance': Scale,
  
  // Tasks & Projects
  'check-square': CheckSquare,
  'CheckSquare': CheckSquare,
  'check-circle': CheckCircle,
  'CheckCircle': CheckCircle,
  'check': CheckCircle,
  'folder': Folder,
  'Folder': Folder,
  'folder-plus': FolderPlus,
  'FolderPlus': FolderPlus,
  'folder-kanban': FolderKanban,
  'FolderKanban': FolderKanban,
  'clipboard': Clipboard,
  'Clipboard': Clipboard,
  'clipboard-list': ClipboardList,
  'ClipboardList': ClipboardList,
  'flag': Flag,
  'Flag': Flag,
  'archive': Archive,
  'Archive': Archive,
  
  // Calendar & Time
  'calendar': Calendar,
  'Calendar': Calendar,
  'calendar-check': CalendarCheck,
  'CalendarCheck': CalendarCheck,
  'calendar-clock': CalendarClock,
  'CalendarClock': CalendarClock,
  'clock': Clock,
  'Clock': Clock,
  'sun': Sun,
  'Sun': Sun,
  'moon': Moon,
  'Moon': Moon,
  'timer': Timer,
  'Timer': Timer,
  
  // Analytics & Charts
  'bar-chart': BarChart2,
  'bar-chart-2': BarChart2,
  'BarChart2': BarChart2,
  'bar-chart-3': BarChart3,
  'BarChart3': BarChart3,
  'chart': BarChart3,
  'pie-chart': PieChart,
  'PieChart': PieChart,
  'trending-up': TrendingUp,
  'TrendingUp': TrendingUp,
  'trending-down': TrendingDown,
  'TrendingDown': TrendingDown,
  'activity': Activity,
  'Activity': Activity,
  
  // AI & Tech
  'bot': Bot,
  'Bot': Bot,
  'robot': Bot,
  'brain': Brain,
  'Brain': Brain,
  'brain-circuit': BrainCircuit,
  'BrainCircuit': BrainCircuit,
  'cpu': Cpu,
  'Cpu': Cpu,
  'server': Server,
  'Server': Server,
  'hard-drive': HardDrive,
  'HardDrive': HardDrive,
  'memory-stick': MemoryStick,
  'MemoryStick': MemoryStick,
  'database': Database,
  'Database': Database,
  'code': Terminal,
  'terminal': Terminal,
  'Terminal': Terminal,
  'workflow': Workflow,
  'Workflow': Workflow,
  'network': Network,
  'Network': Network,
  'plug': Plug,
  'Plug': Plug,
  'sparkles': Sparkles,
  'Sparkles': Sparkles,
  'zap': Zap,
  'Zap': Zap,
  
  // Security & Keys
  'shield': Shield,
  'Shield': Shield,
  'shield-check': ShieldCheck,
  'ShieldCheck': ShieldCheck,
  'shield-alert': ShieldAlert,
  'ShieldAlert': ShieldAlert,
  'lock': Lock,
  'Lock': Lock,
  'key': Key,
  'Key': Key,
  'eye': Eye,
  'Eye': Eye,
  'eye-off': EyeOff,
  'EyeOff': EyeOff,
  
  // Files & Documents
  'file': FileText,
  'file-text': FileText,
  'FileText': FileText,
  'book': Book,
  'Book': Book,
  'book-open': BookOpen,
  'BookOpen': BookOpen,
  'scroll-text': ScrollText,
  'ScrollText': ScrollText,
  
  // Actions
  'plus': Plus,
  'Plus': Plus,
  'edit': Edit,
  'Edit': Edit,
  'edit-2': Edit,
  'trash': XCircle,
  'trash-2': XCircle,
  'download': Download,
  'Download': Download,
  'upload': Upload,
  'Upload': Upload,
  'share': Share2,
  'share-2': Share2,
  'Share2': Share2,
  'link': Link,
  'Link': Link,
  'filter': Filter,
  'Filter': Filter,
  'sliders': Sliders,
  'Sliders': Sliders,
  'search': Search,
  'Search': Search,
  'maximize-2': Maximize2,
  'Maximize2': Maximize2,
  'pen-tool': PenTool,
  'PenTool': PenTool,
  'calculator': Calculator,
  'Calculator': Calculator,
  'refresh': RefreshCw,
  'refresh-cw': RefreshCw,
  'RefreshCw': RefreshCw,
  'rotate-ccw': RotateCcw,
  'RotateCcw': RotateCcw,
  
  // Notifications & Alerts
  'bell': Bell,
  'Bell': Bell,
  'alert-triangle': AlertTriangle,
  'AlertTriangle': AlertTriangle,
  'alert-circle': AlertCircle,
  'AlertCircle': AlertCircle,
  'info': Info,
  'Info': Info,
  
  // Status & Indicators
  'loader': Loader,
  'Loader': Loader,
  'repeat': Repeat,
  'Repeat': Repeat,
  'x-circle': XCircle,
  'XCircle': XCircle,
  
  // Social
  'twitter': Twitter,
  'Twitter': Twitter,
  'linkedin': Linkedin,
  'Linkedin': Linkedin,
  'github': Github,
  'Github': Github,
  'globe': Globe,
  'Globe': Globe,
  
  // Life & Personal
  'heart': Heart,
  'Heart': Heart,
  'star': Star,
  'Star': Star,
  'home': Home,
  'Home': Home,
  'map-pin': MapPin,
  'MapPin': MapPin,
  'target': Target,
  'Target': Target,
  'lightbulb': Lightbulb,
  'Lightbulb': Lightbulb,
  'palette': Palette,
  'Palette': Palette,
  'cloud': Cloud,
  'Cloud': Cloud,
  'laptop': Laptop,
  'Laptop': Laptop,
  'image': Image,
  'Image': Image,
  'plane': Plane,
  'Plane': Plane,
  
  // Settings & Config
  'settings': Settings,
  'Settings': Settings,
  'log-out': LogOut,
  'LogOut': LogOut,
  
  // Misc
  'play': Play,
  'Play': Play,
  'rocket': Rocket,
  'Rocket': Rocket,
  'crown': Crown,
  'Crown': Crown,
  'hammer': Hammer,
  'Hammer': Hammer,
  'box': Box,
  'Box': Box,
  'package': Package,
  'Package': Package,
  'test-tube': TestTube,
  'TestTube': TestTube,
  'git-branch': GitBranch,
  'GitBranch': GitBranch,
  'git-merge': GitMerge,
  'GitMerge': GitMerge,
  'list-todo': ListTodo,
  'ListTodo': ListTodo,
  'list-ordered': ListOrdered,
  'ListOrdered': ListOrdered,
  'dollar': DollarSign,
  'dollar-sign': DollarSign,
  'DollarSign': DollarSign,
  'mic': Mic,
  'Mic': Mic,
  'languages': Languages,
  'Languages': Languages,
  'wifi': Wifi,
  'Wifi': Wifi,
  'help-circle': HelpCircle,
  'HelpCircle': HelpCircle,
  'arrow-up': ArrowUp,
  'ArrowUp': ArrowUp,
  'arrow-down': ArrowDown,
  'ArrowDown': ArrowDown,
  'minus': Minus,
  'Minus': Minus,
  'type': Type,
  'Type': Type,
};

interface IconProps {
  name: string;
  size?: number;
  color?: string;
}

export function LucideIcon({ name, size = 16, color = '#FAFAFA' }: IconProps) {
  const IconComponent = iconMap[name];
  
  if (!IconComponent) {
    // Fallback to Layers icon if not found
    console.warn(`Icon not found: ${name}`);
    return <Layers size={size} color={color} />;
  }
  
  return <IconComponent size={size} color={color} />;
}

// Badge component for notification counts
interface BadgeProps {
  count?: number;
  color?: string;
  size?: 'small' | 'medium';
}

export function Badge({ count, color = '#EF4444', size = 'small' }: BadgeProps) {
  if (count === undefined) return null;
  
  const isSmall = size === 'small';
  const minWidth = isSmall ? 16 : 20;
  const height = isSmall ? 16 : 20;
  const fontSize = isSmall ? 10 : 12;
  
  return (
    <View style={[
      styles.badge,
      { 
        backgroundColor: color,
        minWidth,
        height,
        borderRadius: height / 2,
      }
    ]}>
      <Text style={[styles.badgeText, { fontSize }]}>
        {count > 99 ? '99+' : count}
      </Text>
    </View>
  );
}

// Status dot component
interface StatusDotProps {
  status: 'online' | 'offline' | 'busy' | 'away';
  size?: number;
}

export function StatusDot({ status, size = 8 }: StatusDotProps) {
  const colors = {
    online: '#22C55E',
    offline: '#6B7280',
    busy: '#EF4444',
    away: '#F59E0B',
  };
  
  return (
    <View style={[
      styles.statusDot,
      { 
        backgroundColor: colors[status],
        width: size,
        height: size,
        borderRadius: size / 2,
      }
    ]} />
  );
}

const styles = StyleSheet.create({
  badge: {
    paddingHorizontal: 4,
    justifyContent: 'center',
    alignItems: 'center',
  },
  badgeText: {
    color: '#FFFFFF',
    fontWeight: '600',
  },
  statusDot: {
    borderWidth: 2,
    borderColor: '#000000',
  },
});

export { iconMap };
export default LucideIcon;
