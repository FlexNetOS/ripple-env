import { useState, useEffect } from "react";
import svgPaths from "../imports/svg-svkvdgwod6";
import {
  Search,
  Dashboard,
  Task,
  Folder,
  Calendar,
  UserMultiple,
  Analytics,
  DocumentAdd,
  Settings,
  User,
  ChevronDown,
  ChevronRight,
  OverflowMenuHorizontal,
  CheckmarkOutline,
  Time,
  InProgress,
  Pending,
  Archive,
  Flag,
  AddLarge,
  Filter,
  Renew,
  View,
  Report,
  Share,
  CloudUpload,
  Notification,
  Security,
  Integration,
  StarFilled,
  Group,
  Calendar as CalendarIcon,
  Home,
  ChartBar,
  FolderOpen,
  ChevronLeft,
  ChevronUp,
} from "@carbon/icons-react";
import {
  Facebook,
  Instagram,
  Youtube,
  Twitter,
  Plus,
  Share2,
  Bot,
  Sparkles,
  MessageSquare,
  Target,
  Lightbulb,
  Shield,
  Brain,
  Monitor,
  Laptop,
  Smartphone,
  Tablet,
  Server,
  Network,
  Cpu,
  HardDrive,
  Activity,
  Tv,
  Thermometer,
  Zap,
  Droplets,
  Flame,
  ScanSearch,
  Camera,
  Lock,
  Wifi,
  MapPin,
  ShieldAlert,
  Video,
  Phone,
  Mail,
  Clock,
  Users,
  UserPlus,
  Bell,
  Command,
  Star,
  History,
  TrendingUp,
  Briefcase,
  Heart,
  Wrench,
  BarChart3,
  Layers,
  Eye,
  Radio,
  Gauge,
  Workflow,
  CircleDot,
  Wallet,
  CreditCard,
  DollarSign,
  TrendingDown,
  PiggyBank,
  Receipt,
  Repeat,
  FileText,
  Key,
  FileKey,
  Scale,
  CheckCircle,
  AlertTriangle,
  X,
  Check,
  Trash2,
} from "lucide-react";

// Softer spring animation curve
const softSpringEasing = "cubic-bezier(0.25, 1.1, 0.4, 1)";

// Workspace types
type WorkspaceType = "all" | "productivity" | "communication" | "intelligence" | "infrastructure" | "life" | "business";

interface NotificationBadge {
  count?: number;
  status?: "success" | "warning" | "error" | "info";
  pulse?: boolean;
}

interface NotificationItem {
  id: string;
  title: string;
  message: string;
  time: string;
  type: "alert" | "update" | "mention" | "system";
  read: boolean;
  icon?: React.ReactNode;
}

interface FavoriteItem {
  id: string;
  label: string;
  section: string;
  icon: React.ReactNode;
  badge?: NotificationBadge;
}

function InterfacesLogo1() {
  return (
    <div
      className="aspect-[24/24] basis-0 grow min-h-px min-w-px overflow-clip relative shrink-0"
      data-name="Interfaces Logo"
    >
      <div
        className="absolute aspect-[24/16] left-0 right-0 top-1/2 translate-y-[-50%]"
        data-name="Union"
      >
        <svg
          className="block size-full"
          fill="none"
          preserveAspectRatio="none"
          role="presentation"
          viewBox="0 0 24 16"
        >
          <g id="Union">
            <path
              d={svgPaths.p36880f80}
              fill="var(--fill-0, #FAFAFA)"
              style={{
                fill: "color(display-p3 0.9804 0.9804 0.9804)",
                fillOpacity: "1",
              }}
            />
            <path
              d={svgPaths.p355df480}
              fill="var(--fill-0, #FAFAFA)"
              style={{
                fill: "color(display-p3 0.9804 0.9804 0.9804)",
                fillOpacity: "1",
              }}
            />
            <path
              d={svgPaths.pfa0d600}
              fill="var(--fill-0, #FAFAFA)"
              style={{
                fill: "color(display-p3 0.9804 0.9804 0.9804)",
                fillOpacity: "1",
              }}
            />
          </g>
        </svg>
      </div>
    </div>
  );
}

function Avatar() {
  return (
    <div
      className="bg-[#000000] relative rounded-[999px] shrink-0 size-8"
      data-name="Avatar"
    >
      <div className="box-border content-stretch flex flex-row items-center justify-center overflow-clip p-0 relative size-8">
        <User size={16} className="text-neutral-50" />
      </div>
      <div
        aria-hidden="true"
        className="absolute border border-neutral-800 border-solid inset-0 pointer-events-none rounded-[999px]"
      />
    </div>
  );
}

// Badge component for notifications
function Badge({ count, status, pulse }: NotificationBadge) {
  if (!count && !status) return null;

  const statusColors = {
    success: "bg-green-500",
    warning: "bg-yellow-500",
    error: "bg-red-500",
    info: "bg-blue-500",
  };

  const bgColor = status ? statusColors[status] : "bg-red-500";

  return (
    <div
      className={`absolute -top-1.5 -right-1.5 ${bgColor} rounded-full min-w-[18px] h-[18px] flex items-center justify-center text-[10px] font-semibold text-white px-1.5 shadow-lg border border-neutral-900 ${
        pulse ? "animate-pulse" : ""
      }`}
    >
      {count && count > 99 ? "99+" : count}
    </div>
  );
}

// Status dot for online/offline/etc
function StatusDot({ status }: { status: "online" | "offline" | "away" | "busy" | "good" | "warning" | "error" }) {
  const colors = {
    online: "bg-green-500",
    offline: "bg-gray-500",
    away: "bg-yellow-500",
    busy: "bg-red-500",
    good: "bg-green-500",
    warning: "bg-yellow-500",
    error: "bg-red-500",
  };

  return (
    <div className={`w-2 h-2 rounded-full ${colors[status]} ${status === "online" ? "animate-pulse" : ""}`} />
  );
}

// Notifications Panel
function NotificationsPanel({
  isOpen,
  onClose,
}: {
  isOpen: boolean;
  onClose: () => void;
}) {
  const [notifications, setNotifications] = useState<NotificationItem[]>([
    {
      id: "1",
      title: "Security Alert",
      message: "Garage door left open for 30 minutes",
      time: "5m ago",
      type: "alert",
      read: false,
      icon: <ShieldAlert size={16} className="text-red-400" />,
    },
    {
      id: "2",
      title: "Task Completed",
      message: "Database migration finished successfully",
      time: "15m ago",
      type: "update",
      read: false,
      icon: <CheckCircle size={16} className="text-green-400" />,
    },
    {
      id: "3",
      title: "Meeting Reminder",
      message: "Team standup starts in 30 minutes",
      time: "30m ago",
      type: "update",
      read: false,
      icon: <Clock size={16} className="text-blue-400" />,
    },
    {
      id: "4",
      title: "Energy Spike",
      message: "Unusual energy consumption detected",
      time: "1h ago",
      type: "alert",
      read: true,
      icon: <Zap size={16} className="text-yellow-400" />,
    },
    {
      id: "5",
      title: "Subscription Renewal",
      message: "Netflix subscription renews tomorrow",
      time: "2h ago",
      type: "update",
      read: true,
      icon: <Repeat size={16} className="text-blue-400" />,
    },
    {
      id: "6",
      title: "Family Update",
      message: "Kids arrived at school safely",
      time: "3h ago",
      type: "update",
      read: true,
      icon: <Heart size={16} className="text-pink-400" />,
    },
  ]);

  const markAsRead = (id: string) => {
    setNotifications(prev => prev.map(n => n.id === id ? { ...n, read: true } : n));
  };

  const markAllAsRead = () => {
    setNotifications(prev => prev.map(n => ({ ...n, read: true })));
  };

  const deleteNotification = (id: string) => {
    setNotifications(prev => prev.filter(n => n.id !== id));
  };

  const unreadCount = notifications.filter(n => !n.read).length;

  if (!isOpen) return null;

  return (
    <>
      <div
        className="fixed inset-0 bg-black/20 backdrop-blur-sm z-40"
        onClick={onClose}
      />
      <div className="fixed right-4 top-4 bottom-4 w-[400px] bg-neutral-900 rounded-xl border border-neutral-800 shadow-2xl z-50 flex flex-col">
        {/* Header */}
        <div className="p-4 border-b border-neutral-800">
          <div className="flex items-center justify-between mb-3">
            <div className="flex items-center gap-2">
              <Bell size={20} className="text-neutral-50" />
              <h3 className="font-['Lexend:SemiBold',_sans-serif] font-semibold text-[16px] text-neutral-50">
                Notifications
              </h3>
              {unreadCount > 0 && (
                <span className="px-2 py-0.5 bg-red-500 rounded-full text-[11px] text-white font-semibold">
                  {unreadCount}
                </span>
              )}
            </div>
            <button
              onClick={onClose}
              className="p-1.5 hover:bg-neutral-800 rounded-lg transition-colors"
            >
              <X size={18} className="text-neutral-400" />
            </button>
          </div>
          {unreadCount > 0 && (
            <button
              onClick={markAllAsRead}
              className="text-[13px] text-blue-400 hover:text-blue-300 transition-colors"
            >
              Mark all as read
            </button>
          )}
        </div>

        {/* Notifications List */}
        <div className="flex-1 overflow-y-auto p-2">
          {notifications.length === 0 ? (
            <div className="flex flex-col items-center justify-center h-full text-neutral-500">
              <Bell size={48} className="mb-3 opacity-30" />
              <p>No notifications</p>
            </div>
          ) : (
            notifications.map((notification) => (
              <div
                key={notification.id}
                className={`p-3 mb-2 rounded-lg transition-colors cursor-pointer group ${
                  notification.read
                    ? "bg-neutral-950/50 hover:bg-neutral-800"
                    : "bg-neutral-800 hover:bg-neutral-700"
                }`}
                onClick={() => markAsRead(notification.id)}
              >
                <div className="flex gap-3">
                  <div className="mt-1">{notification.icon}</div>
                  <div className="flex-1 min-w-0">
                    <div className="flex items-start justify-between gap-2 mb-1">
                      <h4 className={`text-[14px] font-medium ${
                        notification.read ? "text-neutral-400" : "text-neutral-50"
                      }`}>
                        {notification.title}
                      </h4>
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          deleteNotification(notification.id);
                        }}
                        className="opacity-0 group-hover:opacity-100 p-1 hover:bg-neutral-900 rounded transition-all"
                      >
                        <Trash2 size={14} className="text-neutral-500" />
                      </button>
                    </div>
                    <p className="text-[13px] text-neutral-500 mb-2">
                      {notification.message}
                    </p>
                    <span className="text-[11px] text-neutral-600">
                      {notification.time}
                    </span>
                  </div>
                </div>
              </div>
            ))
          )}
        </div>
      </div>
    </>
  );
}

// Favorites Panel
function FavoritesPanel({
  isOpen,
  onClose,
  onNavigate,
}: {
  isOpen: boolean;
  onClose: () => void;
  onNavigate: (section: string) => void;
}) {
  const [favorites, setFavorites] = useState<FavoriteItem[]>([
    {
      id: "1",
      label: "AI Command Center",
      section: "ai",
      icon: <Bot size={16} className="text-purple-400" />,
      badge: { count: 3, status: "info" },
    },
    {
      id: "2",
      label: "Life Dashboard",
      section: "dashboard",
      icon: <Dashboard size={16} className="text-blue-400" />,
    },
    {
      id: "3",
      label: "Wallet",
      section: "wallet",
      icon: <Wallet size={16} className="text-green-400" />,
      badge: { count: 2, status: "warning" },
    },
    {
      id: "4",
      label: "Security",
      section: "security",
      icon: <ShieldAlert size={16} className="text-red-400" />,
      badge: { status: "success" },
    },
    {
      id: "5",
      label: "Smart Home",
      section: "smarthome",
      icon: <Home size={16} className="text-orange-400" />,
    },
  ]);

  const removeFavorite = (id: string) => {
    setFavorites(prev => prev.filter(f => f.id !== id));
  };

  if (!isOpen) return null;

  return (
    <>
      <div
        className="fixed inset-0 bg-black/20 backdrop-blur-sm z-40"
        onClick={onClose}
      />
      <div className="fixed right-4 top-4 bottom-4 w-[360px] bg-neutral-900 rounded-xl border border-neutral-800 shadow-2xl z-50 flex flex-col">
        {/* Header */}
        <div className="p-4 border-b border-neutral-800">
          <div className="flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Star size={20} className="text-yellow-400" />
              <h3 className="font-['Lexend:SemiBold',_sans-serif] font-semibold text-[16px] text-neutral-50">
                Favorites
              </h3>
            </div>
            <button
              onClick={onClose}
              className="p-1.5 hover:bg-neutral-800 rounded-lg transition-colors"
            >
              <X size={18} className="text-neutral-400" />
            </button>
          </div>
          <p className="text-[12px] text-neutral-500 mt-2">
            Quick access to your most used sections
          </p>
        </div>

        {/* Favorites List */}
        <div className="flex-1 overflow-y-auto p-3">
          {favorites.length === 0 ? (
            <div className="flex flex-col items-center justify-center h-full text-neutral-500">
              <Star size={48} className="mb-3 opacity-30" />
              <p>No favorites yet</p>
              <p className="text-[12px] mt-1">Star items to add them here</p>
            </div>
          ) : (
            <div className="space-y-2">
              {favorites.map((favorite) => (
                <div
                  key={favorite.id}
                  className="p-3 bg-neutral-800 hover:bg-neutral-700 rounded-lg transition-colors cursor-pointer group flex items-center justify-between relative"
                  onClick={() => {
                    onNavigate(favorite.section);
                    onClose();
                  }}
                >
                  <div className="flex items-center gap-3 flex-1 min-w-0 mr-2">
                    <div className="flex-shrink-0 relative">
                      {favorite.icon}
                      {favorite.badge && <Badge {...favorite.badge} />}
                    </div>
                    <span className="text-[14px] text-neutral-50 font-medium truncate">
                      {favorite.label}
                    </span>
                  </div>
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      removeFavorite(favorite.id);
                    }}
                    className="opacity-0 group-hover:opacity-100 p-1.5 hover:bg-neutral-900 rounded transition-all flex-shrink-0"
                  >
                    <X size={14} className="text-neutral-500" />
                  </button>
                </div>
              ))}
            </div>
          )}
        </div>

        {/* Footer */}
        <div className="p-3 border-t border-neutral-800">
          <p className="text-[11px] text-neutral-600 text-center">
            Right-click any section to add to favorites
          </p>
        </div>
      </div>
    </>
  );
}

function SearchContainer({
  isCollapsed = false,
  onQuickSwitcher,
}: {
  isCollapsed?: boolean;
  onQuickSwitcher?: () => void;
}) {
  const [searchValue, setSearchValue] = useState("");

  return (
    <div
      className={`relative shrink-0 transition-all duration-500 ${
        isCollapsed ? "w-full flex justify-center" : "w-full"
      }`}
      style={{ transitionTimingFunction: softSpringEasing }}
      data-name="Search Container"
    >
      <div
        className={`bg-[#000000] h-10 relative rounded-lg flex items-center transition-all duration-500 cursor-pointer ${
          isCollapsed
            ? "w-10 min-w-10 justify-center hover:bg-neutral-900"
            : "w-full"
        }`}
        style={{ transitionTimingFunction: softSpringEasing }}
        onClick={isCollapsed ? onQuickSwitcher : undefined}
      >
        <div
          className={`flex items-center justify-center shrink-0 transition-all duration-500 ${
            isCollapsed ? "p-1" : "px-1"
          }`}
          style={{ transitionTimingFunction: softSpringEasing }}
        >
          <div className="size-8 flex items-center justify-center">
            <Search size={16} className="text-neutral-50" />
          </div>
        </div>
        <div
          className={`flex-1 min-h-px min-w-px relative transition-opacity duration-500 overflow-hidden ${
            isCollapsed ? "opacity-0 w-0" : "opacity-100"
          }`}
          style={{ transitionTimingFunction: softSpringEasing }}
        >
          <div className="flex flex-col justify-center relative size-full">
            <div className="box-border content-stretch flex flex-col gap-2 items-start justify-center pl-0 pr-2 py-1 relative w-full">
              <input
                type="text"
                placeholder="Search or press ⌘K..."
                value={searchValue}
                onChange={(e) => setSearchValue(e.target.value)}
                onKeyDown={(e) => {
                  if ((e.metaKey || e.ctrlKey) && e.key === "k") {
                    e.preventDefault();
                    onQuickSwitcher?.();
                  }
                }}
                className="w-full bg-transparent border-none outline-none font-['Lexend:Regular',_sans-serif] font-normal text-[14px] text-neutral-50 placeholder:text-neutral-400 leading-[20px]"
                tabIndex={isCollapsed ? -1 : 0}
              />
            </div>
          </div>
        </div>
        {!isCollapsed && (
          <div className="pr-2 flex items-center gap-1">
            <kbd className="px-1.5 py-0.5 bg-neutral-900 rounded text-[10px] text-neutral-400 border border-neutral-800">
              ⌘K
            </kbd>
          </div>
        )}
        <div
          aria-hidden="true"
          className="absolute border border-neutral-800 border-solid inset-0 pointer-events-none rounded-lg"
        />
      </div>
    </div>
  );
}

interface MenuItem {
  icon: React.ReactNode;
  label: string;
  hasDropdown?: boolean;
  isActive?: boolean;
  status?: "online" | "offline" | "away" | "busy" | "good" | "warning" | "error";
  badge?: NotificationBadge;
  children?: MenuItem[];
  metadata?: string;
  shortcut?: string;
}

interface MenuSection {
  title: string;
  items: MenuItem[];
}

interface SidebarContent {
  title: string;
  sections: MenuSection[];
  headerBadge?: NotificationBadge;
}

function MenuItem({
  item,
  isExpanded,
  onToggle,
  onItemClick,
  isCollapsed,
}: {
  item: MenuItem;
  isExpanded?: boolean;
  onToggle?: () => void;
  onItemClick?: () => void;
  isCollapsed?: boolean;
}) {
  const handleClick = () => {
    if (item.hasDropdown && onToggle) {
      onToggle();
    } else if (onItemClick) {
      onItemClick();
    }
  };

  return (
    <div
      className={`relative shrink-0 transition-all duration-500 ${
        isCollapsed ? "w-full flex justify-center" : "w-full"
      }`}
      style={{ transitionTimingFunction: softSpringEasing }}
    >
      <div
        className={`select-none rounded-lg cursor-pointer transition-all duration-500 flex items-center relative my-1 group ${
          item.isActive
            ? "bg-neutral-900"
            : "hover:bg-neutral-900"
        } ${
          isCollapsed
            ? "w-10 min-w-10 h-10 justify-center p-4"
            : "w-full h-11 px-4 py-2.5"
        }`}
        style={{ transitionTimingFunction: softSpringEasing }}
        onClick={handleClick}
        title={isCollapsed ? item.label : undefined}
      >
        <div className="flex items-center justify-center shrink-0 relative">
          {item.icon}
          {item.status && (
            <div className="absolute -bottom-0.5 -right-0.5">
              <StatusDot status={item.status} />
            </div>
          )}
          {item.badge && isCollapsed && (
            <Badge {...item.badge} />
          )}
        </div>
        <div
          className={`flex-1 min-h-px min-w-px relative transition-opacity duration-500 overflow-hidden ${
            isCollapsed ? "opacity-0 w-0" : "opacity-100 ml-3"
          }`}
          style={{ transitionTimingFunction: softSpringEasing }}
        >
          <div className="flex items-center justify-between w-full">
            <div className="flex flex-col justify-center flex-1 min-w-0 mr-2">
              <div className="font-['Lexend:Medium',_sans-serif] font-medium text-[14px] text-neutral-50 leading-[20px] truncate">
                {item.label}
              </div>
              {item.metadata && (
                <div className="font-['Lexend:Regular',_sans-serif] font-normal text-[11px] text-neutral-500 leading-[15px] truncate mt-0.5">
                  {item.metadata}
                </div>
              )}
            </div>
            {item.badge && (
              <div className="flex-shrink-0 relative">
                <Badge {...item.badge} />
              </div>
            )}
          </div>
        </div>
        {item.hasDropdown && (
          <div
            className={`flex items-center justify-center shrink-0 transition-opacity duration-500 ${
              isCollapsed ? "opacity-0 w-0" : "opacity-100 ml-2"
            }`}
            style={{
              transitionTimingFunction: softSpringEasing,
            }}
          >
            <ChevronDown
              size={16}
              className={`text-neutral-50 transition-transform duration-500`}
              style={{
                transitionTimingFunction: softSpringEasing,
                transform: isExpanded
                  ? "rotate(180deg)"
                  : "rotate(0deg)",
              }}
            />
          </div>
        )}
        {item.shortcut && !isCollapsed && (
          <div className="ml-auto pl-2 opacity-0 group-hover:opacity-100 transition-opacity">
            <kbd className="px-1.5 py-0.5 bg-neutral-900 rounded text-[10px] text-neutral-400 border border-neutral-800">
              {item.shortcut}
            </kbd>
          </div>
        )}
      </div>
    </div>
  );
}

function SubMenuItem({
  item,
  onItemClick,
}: {
  item: MenuItem;
  onItemClick?: () => void;
}) {
  return (
    <div className="select-none w-full pl-9 pr-1 py-0.5">
      <div
        className="h-10 w-full rounded-lg cursor-pointer transition-colors hover:bg-neutral-900 flex items-center px-3 py-2 group relative"
        onClick={onItemClick}
      >
        <div className="flex-1 min-w-0 flex items-center gap-2">
          {item.status && <StatusDot status={item.status} />}
          <div className="flex-1 min-w-0 mr-2">
            <div className="font-['Lexend:Regular',_sans-serif] font-normal text-[13px] text-neutral-300 leading-[18px] truncate">
              {item.label}
            </div>
            {item.metadata && (
              <div className="font-['Lexend:Regular',_sans-serif] font-normal text-[11px] text-neutral-500 leading-[14px] truncate mt-0.5">
                {item.metadata}
              </div>
            )}
          </div>
          {item.badge && (
            <div className="flex-shrink-0 relative">
              <Badge {...item.badge} />
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

function MenuSection({
  section,
  expandedItems,
  onToggleExpanded,
  isCollapsed,
}: {
  section: MenuSection;
  expandedItems: Set<string>;
  onToggleExpanded: (itemKey: string) => void;
  isCollapsed?: boolean;
}) {
  return (
    <div className="box-border content-stretch flex flex-col items-start justify-stretch p-0 relative shrink-0 w-full">
      <div
        className={`relative shrink-0 w-full transition-all duration-500 overflow-hidden ${
          isCollapsed ? "h-0 opacity-0" : "min-h-[44px] opacity-100"
        }`}
        style={{ transitionTimingFunction: softSpringEasing }}
      >
        <div className="flex flex-col justify-center relative size-full">
          <div className="box-border content-stretch flex flex-col min-h-[44px] items-start justify-center px-4 py-3 relative w-full">
            <div className="font-['Lexend:SemiBold',_sans-serif] font-semibold leading-[0] relative shrink-0 text-[12px] text-left text-neutral-400 text-nowrap uppercase tracking-wider">
              <p className="block leading-[18px] whitespace-pre">
                {section.title}
              </p>
            </div>
          </div>
        </div>
      </div>
      {section.items.map((item, index) => {
        const itemKey = `${section.title}-${index}`;
        const isExpanded = expandedItems.has(itemKey);
        return (
          <div
            key={itemKey}
            className="w-full flex flex-col content-stretch"
          >
            <MenuItem
              item={item}
              isExpanded={isExpanded}
              onToggle={() => onToggleExpanded(itemKey)}
              onItemClick={() =>
                console.log(`Clicked ${item.label}`)
              }
              isCollapsed={isCollapsed}
            />
            {isExpanded && item.children && !isCollapsed && (
              <div className="flex flex-col gap-0.5 mb-2 mt-1">
                {item.children.map((child, childIndex) => (
                  <SubMenuItem
                    key={`${itemKey}-${childIndex}`}
                    item={child}
                    onItemClick={() =>
                      console.log(`Clicked ${child.label}`)
                    }
                  />
                ))}
              </div>
            )}
          </div>
        );
      })}
    </div>
  );
}

function getSidebarContent(
  activeSection: string,
): SidebarContent {
  const contentMap: Record<string, SidebarContent> = {
    // AI COMMAND CENTER - The brain that runs everything
    ai: {
      title: "AI Command Center",
      headerBadge: { count: 3, status: "info" },
      sections: [
        {
          title: "Active Conversations",
          items: [
            {
              icon: <MessageSquare size={16} className="text-neutral-50" />,
              label: "Life Assistant",
              badge: { count: 2, pulse: true },
              metadata: "Running your day",
              hasDropdown: true,
              shortcut: "⌘1",
              children: [
                { label: "Morning routine optimization", metadata: "2m ago" },
                { label: "Family schedule coordination", metadata: "5m ago" },
              ],
            },
            {
              icon: <MessageSquare size={16} className="text-neutral-50" />,
              label: "Work Strategy",
              metadata: "Project planning",
              hasDropdown: true,
              children: [
                { label: "Q1 goals review", metadata: "Active now" },
                { label: "Team performance insights", metadata: "1h ago" },
              ],
            },
            {
              icon: <MessageSquare size={16} className="text-neutral-50" />,
              label: "Home Automation",
              metadata: "Managing smart home",
              hasDropdown: true,
              children: [
                { label: "Evening scene setup", metadata: "Scheduled 6 PM" },
                { label: "Energy optimization tips", metadata: "Today" },
              ],
            },
          ],
        },
        {
          title: "AI Knowledge Base",
          items: [
            {
              icon: <Target size={16} className="text-neutral-50" />,
              label: "Goals & Objectives",
              badge: { count: 8 },
              hasDropdown: true,
              children: [
                { label: "Complete Q1 product launch", status: "good", metadata: "85% complete" },
                { label: "Family vacation planning", status: "warning", metadata: "Needs attention" },
                { label: "Home security upgrade", status: "good", metadata: "On track" },
                { label: "Health & fitness targets", status: "good", metadata: "Meeting goals" },
              ],
            },
            {
              icon: <Lightbulb size={16} className="text-neutral-50" />,
              label: "Ideas & Insights",
              badge: { count: 12 },
              hasDropdown: true,
              children: [
                { label: "Weekend automation ideas", metadata: "AI suggested" },
                { label: "Cost-saving strategies", metadata: "High priority" },
                { label: "Family time optimization", metadata: "New" },
              ],
            },
            {
              icon: <Shield size={16} className="text-neutral-50" />,
              label: "Rules & Protocols",
              hasDropdown: true,
              children: [
                { label: "Work-life balance rules", status: "good" },
                { label: "Family safety protocols", status: "good" },
                { label: "Security guidelines", status: "good" },
                { label: "Energy efficiency rules", status: "good" },
              ],
            },
            {
              icon: <Brain size={16} className="text-neutral-50" />,
              label: "Memory & Context",
              hasDropdown: true,
              children: [
                { label: "Preferences & habits", metadata: "128 entries" },
                { label: "Relationship context", metadata: "Family & work" },
                { label: "Decision history", metadata: "Last 90 days" },
                { label: "Life patterns", metadata: "AI analyzed" },
              ],
            },
          ],
        },
        {
          title: "AI Capabilities",
          items: [
            {
              icon: <Sparkles size={16} className="text-neutral-50" />,
              label: "Multi-Modal Interface",
              hasDropdown: true,
              children: [
                { label: "Voice commands", status: "good" },
                { label: "Visual recognition", status: "good" },
                { label: "Text generation", status: "good" },
                { label: "Real-time translation", status: "good" },
              ],
            },
            {
              icon: <Bot size={16} className="text-neutral-50" />,
              label: "Specialized Assistants",
              hasDropdown: true,
              children: [
                { label: "Work productivity coach", status: "online" },
                { label: "Family coordinator", status: "online" },
                { label: "Health advisor", status: "online" },
                { label: "Financial planner", status: "online" },
                { label: "Home automation expert", status: "online" },
              ],
            },
            {
              icon: <Workflow size={16} className="text-neutral-50" />,
              label: "Automation & Workflows",
              badge: { count: 15, status: "success" },
              hasDropdown: true,
              children: [
                { label: "Morning routine", status: "good", metadata: "Active" },
                { label: "Work mode trigger", status: "good", metadata: "Active" },
                { label: "Family time mode", status: "good", metadata: "Scheduled" },
                { label: "Sleep optimization", status: "good", metadata: "Active" },
              ],
            },
          ],
        },
        {
          title: "Quick Actions",
          items: [
            {
              icon: <Plus size={16} className="text-neutral-50" />,
              label: "New conversation",
              shortcut: "⌘N",
            },
            {
              icon: <Sparkles size={16} className="text-neutral-50" />,
              label: "Generate UI",
              shortcut: "⌘G",
            },
          ],
        },
      ],
    },
    
    // UNIFIED DASHBOARD
    dashboard: {
      title: "Life Dashboard",
      sections: [
        {
          title: "Overview",
          items: [
            {
              icon: <View size={16} className="text-neutral-50" />,
              label: "Today's Summary",
              isActive: true,
              metadata: "All areas",
              shortcut: "⌘D",
            },
            {
              icon: <TrendingUp size={16} className="text-neutral-50" />,
              label: "Key Metrics",
              hasDropdown: true,
              children: [
                { label: "Work productivity: 87%", status: "good" },
                { label: "Health score: 92%", status: "good" },
                { label: "Home efficiency: 78%", status: "warning" },
                { label: "Family time: 6.5h today", status: "good" },
              ],
            },
          ],
        },
        {
          title: "Domain Dashboards",
          items: [
            {
              icon: <Briefcase size={16} className="text-neutral-50" />,
              label: "Work & Productivity",
              badge: { count: 5 },
              hasDropdown: true,
              children: [
                { label: "Active projects: 3", metadata: "2 due this week" },
                { label: "Tasks completed today: 8/12", status: "good" },
                { label: "Meetings: 4 remaining", metadata: "Next: 2 PM" },
                { label: "Team performance: High", status: "good" },
              ],
            },
            {
              icon: <Heart size={16} className="text-neutral-50" />,
              label: "Family & Personal",
              badge: { count: 2, status: "info" },
              hasDropdown: true,
              children: [
                { label: "Family calendar: 3 events today", metadata: "Soccer at 4 PM" },
                { label: "Kids location: At school", status: "good" },
                { label: "Family time today: 2.5h", status: "good" },
                { label: "Personal goals: On track", status: "good" },
              ],
            },
            {
              icon: <Home size={16} className="text-neutral-50" />,
              label: "Home & Environment",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Security: All clear", status: "good" },
                { label: "Energy usage: Normal", status: "good" },
                { label: "Climate: 72°F", status: "good" },
                { label: "Active devices: 18", metadata: "All online" },
              ],
            },
            {
              icon: <BarChart3 size={16} className="text-neutral-50" />,
              label: "Analytics Hub",
              hasDropdown: true,
              children: [
                { label: "Performance trends", metadata: "Last 30 days" },
                { label: "Spending analysis", metadata: "This month" },
                { label: "Time allocation", metadata: "Work vs Life" },
                { label: "Predictive insights", metadata: "AI powered" },
              ],
            },
          ],
        },
        {
          title: "Recent Activity",
          items: [
            {
              icon: <History size={16} className="text-neutral-50" />,
              label: "Activity Feed",
              badge: { count: 8, pulse: true },
              hasDropdown: true,
              children: [
                { label: "Task completed: Review docs", metadata: "2m ago" },
                { label: "Security alert: Front door", metadata: "5m ago", status: "warning" },
                { label: "Energy spike detected", metadata: "10m ago", status: "warning" },
                { label: "Meeting started: Team sync", metadata: "15m ago" },
              ],
            },
          ],
        },
      ],
    },

    // CONTACTS (renamed from People)
    contacts: {
      title: "Contacts",
      headerBadge: { count: 7, status: "info" },
      sections: [
        {
          title: "Communication",
          items: [
            {
              icon: <Video size={16} className="text-neutral-50" />,
              label: "Video Conferencing",
              badge: { count: 1, status: "warning" },
              hasDropdown: true,
              shortcut: "⌘V",
              children: [
                { label: "Start new meeting" },
                { label: "Join meeting" },
                { label: "Upcoming: Team standup", metadata: "In 30 min", status: "warning" },
                { label: "Recent: Client call", metadata: "2h ago" },
              ],
            },
            {
              icon: <Phone size={16} className="text-neutral-50" />,
              label: "Calls",
              badge: { count: 2 },
              hasDropdown: true,
              children: [
                { label: "Voicemail: 2 new", badge: { count: 2 } },
                { label: "Recent: John Doe", metadata: "10m ago" },
                { label: "Recent: Sarah Wilson", metadata: "1h ago" },
                { label: "Call history" },
              ],
            },
            {
              icon: <MessageSquare size={16} className="text-neutral-50" />,
              label: "Messaging",
              badge: { count: 5, pulse: true },
              hasDropdown: true,
              shortcut: "⌘M",
              children: [
                { label: "Development Team", badge: { count: 3 }, metadata: "3 unread" },
                { label: "Family Group", badge: { count: 2 }, metadata: "2 unread" },
                { label: "Design Team", metadata: "All read" },
                { label: "Direct messages", metadata: "12 conversations" },
              ],
            },
          ],
        },
        {
          title: "Work Contacts",
          items: [
            {
              icon: <Users size={16} className="text-neutral-50" />,
              label: "Development Team",
              badge: { count: 4, status: "success" },
              hasDropdown: true,
              children: [
                { label: "John Doe (Lead)", status: "online", metadata: "NYC · 2:30 PM" },
                { label: "Jane Smith", status: "busy", metadata: "London · 7:30 PM" },
                { label: "Mike Johnson", status: "away", metadata: "Tokyo · 3:30 AM" },
                { label: "Emma Davis", status: "online", metadata: "SF · 11:30 AM" },
              ],
            },
            {
              icon: <Users size={16} className="text-neutral-50" />,
              label: "Design Team",
              badge: { count: 3, status: "success" },
              hasDropdown: true,
              children: [
                { label: "Sarah Wilson", status: "online", metadata: "LA · 11:30 AM" },
                { label: "Tom Brown", status: "offline", metadata: "Berlin · 8:30 PM" },
                { label: "Lisa Chen", status: "away", metadata: "Singapore · 2:30 AM" },
              ],
            },
          ],
        },
        {
          title: "Family & Personal",
          items: [
            {
              icon: <Heart size={16} className="text-neutral-50" />,
              label: "Family",
              hasDropdown: true,
              children: [
                { label: "Partner", status: "online", metadata: "At home · Nearby" },
                { label: "Kids", status: "good", metadata: "At school · Safe zone" },
                { label: "Parents", status: "online", metadata: "Boston · 3:30 PM" },
              ],
            },
            {
              icon: <Star size={16} className="text-neutral-50" />,
              label: "Favorites",
              hasDropdown: true,
              children: [
                { label: "Alex Martinez", status: "online", metadata: "Paris · 8:30 PM" },
                { label: "Chris Lee", status: "offline", metadata: "Sydney · 5:30 AM" },
                { label: "Pat Taylor", status: "online", metadata: "Chicago · 1:30 PM" },
              ],
            },
            {
              icon: <User size={16} className="text-neutral-50" />,
              label: "All Contacts",
              badge: { count: 42 },
              hasDropdown: true,
              children: [
                { label: "Recent: David Kim", metadata: "Messaged 1h ago" },
                { label: "Recent: Maria Garcia", metadata: "Called yesterday" },
                { label: "Browse all contacts" },
              ],
            },
          ],
        },
        {
          title: "Location & Presence",
          items: [
            {
              icon: <MapPin size={16} className="text-neutral-50" />,
              label: "People Nearby",
              hasDropdown: true,
              children: [
                { label: "At home: 2 people", status: "good" },
                { label: "At office: 8 people", status: "good" },
                { label: "Safe zones: All clear", status: "good" },
              ],
            },
            {
              icon: <Clock size={16} className="text-neutral-50" />,
              label: "World Clock",
              hasDropdown: true,
              children: [
                { label: "New York: 2:30 PM", metadata: "John, Emma" },
                { label: "London: 7:30 PM", metadata: "Jane" },
                { label: "Tokyo: 3:30 AM", metadata: "Mike" },
                { label: "San Francisco: 11:30 AM", metadata: "Sarah" },
              ],
            },
          ],
        },
        {
          title: "Quick Actions",
          items: [
            {
              icon: <UserPlus size={16} className="text-neutral-50" />,
              label: "Add new contact",
              shortcut: "⌘⇧A",
            },
          ],
        },
      ],
    },

    // LEGAL & COMPLIANCE - NEW SECTION
    legal: {
      title: "Legal & Compliance",
      headerBadge: { count: 3, status: "warning" },
      sections: [
        {
          title: "Contracts & Agreements",
          items: [
            {
              icon: <FileText size={16} className="text-neutral-50" />,
              label: "Active Contracts",
              badge: { count: 8, status: "success" },
              hasDropdown: true,
              children: [
                { label: "Employment Contract", status: "good", metadata: "Expires: 2026" },
                { label: "Vendor Agreement - AWS", status: "good", metadata: "Annual renewal" },
                { label: "NDA - Client X", status: "good", metadata: "Valid 3 years" },
                { label: "Service Level Agreement", status: "good", metadata: "Review quarterly" },
              ],
            },
            {
              icon: <AlertTriangle size={16} className="text-neutral-50" />,
              label: "Pending Review",
              badge: { count: 3, status: "warning" },
              hasDropdown: true,
              children: [
                { label: "Lease Renewal", status: "warning", metadata: "Due in 30 days" },
                { label: "Insurance Policy Update", status: "warning", metadata: "Needs signature" },
                { label: "Partnership Agreement", status: "warning", metadata: "Review needed" },
              ],
            },
            {
              icon: <CheckCircle size={16} className="text-neutral-50" />,
              label: "Signed Documents",
              badge: { count: 24 },
              hasDropdown: true,
              children: [
                { label: "This year: 12 documents", metadata: "All archived" },
                { label: "Last year: 18 documents", metadata: "All archived" },
                { label: "View all signed contracts" },
              ],
            },
          ],
        },
        {
          title: "Secrets & Keys",
          items: [
            {
              icon: <Key size={16} className="text-neutral-50" />,
              label: "API Keys & Credentials",
              badge: { count: 15, status: "success" },
              hasDropdown: true,
              children: [
                { label: "AWS Access Keys", status: "good", metadata: "Rotated 30d ago" },
                { label: "GitHub Personal Token", status: "good", metadata: "Rotated 15d ago" },
                { label: "Stripe API Keys", status: "good", metadata: "Rotated 45d ago" },
                { label: "OpenAI API Key", status: "warning", metadata: "Rotate soon" },
              ],
            },
            {
              icon: <Lock size={16} className="text-neutral-50" />,
              label: "Password Vault",
              badge: { count: 48 },
              hasDropdown: true,
              children: [
                { label: "Work accounts: 18", metadata: "All secure" },
                { label: "Personal accounts: 22", metadata: "All secure" },
                { label: "Family accounts: 8", metadata: "All secure" },
                { label: "Weak passwords: 0", status: "good" },
              ],
            },
            {
              icon: <FileKey size={16} className="text-neutral-50" />,
              label: "Certificates & Keys",
              hasDropdown: true,
              children: [
                { label: "SSL Certificates: 3", status: "good", metadata: "All valid" },
                { label: "SSH Keys: 5", status: "good", metadata: "All active" },
                { label: "PGP Keys: 2", status: "good", metadata: "All valid" },
              ],
            },
          ],
        },
        {
          title: "Governance & Compliance",
          items: [
            {
              icon: <Scale size={16} className="text-neutral-50" />,
              label: "Compliance Status",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "GDPR Compliance", status: "good", metadata: "Audit passed" },
                { label: "SOC 2 Type II", status: "good", metadata: "Certified" },
                { label: "HIPAA Compliance", status: "good", metadata: "Compliant" },
                { label: "ISO 27001", status: "warning", metadata: "Renewal due" },
              ],
            },
            {
              icon: <Shield size={16} className="text-neutral-50" />,
              label: "Policies & Procedures",
              hasDropdown: true,
              children: [
                { label: "Privacy Policy", metadata: "Last updated: 2024" },
                { label: "Data Retention Policy", metadata: "Version 2.1" },
                { label: "Security Policy", metadata: "Reviewed quarterly" },
                { label: "Acceptable Use Policy", metadata: "Version 3.0" },
              ],
            },
            {
              icon: <CheckCircle size={16} className="text-neutral-50" />,
              label: "Audits & Assessments",
              hasDropdown: true,
              children: [
                { label: "Last security audit", metadata: "3 months ago", status: "good" },
                { label: "Next compliance review", metadata: "In 2 weeks", status: "warning" },
                { label: "Risk assessment", metadata: "Quarterly", status: "good" },
              ],
            },
          ],
        },
        {
          title: "Quick Actions",
          items: [
            {
              icon: <Plus size={16} className="text-neutral-50" />,
              label: "Upload new contract",
            },
            {
              icon: <Key size={16} className="text-neutral-50" />,
              label: "Add new secret",
            },
          ],
        },
      ],
    },

    // WALLET - NEW SECTION
    wallet: {
      title: "Wallet",
      headerBadge: { count: 5, status: "warning" },
      sections: [
        {
          title: "Banking",
          items: [
            {
              icon: <DollarSign size={16} className="text-neutral-50" />,
              label: "Bank Accounts",
              hasDropdown: true,
              children: [
                { label: "Chase Checking", status: "good", metadata: "$12,450.32" },
                { label: "Wells Fargo Savings", status: "good", metadata: "$45,200.00" },
                { label: "Business Account", status: "good", metadata: "$28,150.75" },
                { label: "Total Balance: $85,801.07", metadata: "Updated 2m ago" },
              ],
            },
            {
              icon: <TrendingUp size={16} className="text-neutral-50" />,
              label: "Cash Flow",
              hasDropdown: true,
              children: [
                { label: "Income this month: $8,500", status: "good" },
                { label: "Expenses this month: $6,240", status: "good" },
                { label: "Net: +$2,260", status: "good", metadata: "+36% vs last month" },
              ],
            },
          ],
        },
        {
          title: "Credit Cards",
          items: [
            {
              icon: <CreditCard size={16} className="text-neutral-50" />,
              label: "Credit Cards",
              badge: { count: 2, status: "warning" },
              hasDropdown: true,
              children: [
                { label: "Chase Sapphire Reserve", status: "good", metadata: "$1,240 / $20,000" },
                { label: "AmEx Platinum", status: "warning", metadata: "$4,850 / $15,000" },
                { label: "Citi Double Cash", status: "good", metadata: "$320 / $10,000" },
                { label: "Total utilization: 14%", status: "good" },
              ],
            },
            {
              icon: <Receipt size={16} className="text-neutral-50" />,
              label: "Upcoming Payments",
              badge: { count: 2, status: "warning" },
              hasDropdown: true,
              children: [
                { label: "Chase Sapphire", status: "warning", metadata: "Due in 3 days: $1,240" },
                { label: "AmEx Platinum", status: "warning", metadata: "Due in 5 days: $4,850" },
              ],
            },
          ],
        },
        {
          title: "Subscriptions",
          items: [
            {
              icon: <Repeat size={16} className="text-neutral-50" />,
              label: "Active Subscriptions",
              badge: { count: 12 },
              hasDropdown: true,
              children: [
                { label: "Netflix", metadata: "$15.99/mo · Renews Jan 15" },
                { label: "Spotify Family", metadata: "$16.99/mo · Renews Jan 8" },
                { label: "Adobe Creative Cloud", metadata: "$54.99/mo · Renews Jan 12" },
                { label: "AWS Services", metadata: "$287.50/mo · Variable" },
                { label: "View all 12 subscriptions" },
              ],
            },
            {
              icon: <AlertTriangle size={16} className="text-neutral-50" />,
              label: "Upcoming Renewals",
              badge: { count: 3, status: "warning" },
              hasDropdown: true,
              children: [
                { label: "Spotify Family", status: "warning", metadata: "Renews in 3 days" },
                { label: "Adobe CC", status: "warning", metadata: "Renews in 7 days" },
                { label: "Netflix", status: "warning", metadata: "Renews in 10 days" },
              ],
            },
          ],
        },
        {
          title: "Investments",
          items: [
            {
              icon: <TrendingUp size={16} className="text-neutral-50" />,
              label: "Portfolio Overview",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Total value: $185,450", status: "good", metadata: "+12.5% YTD" },
                { label: "Stocks: $95,200", status: "good", metadata: "+15% YTD" },
                { label: "Bonds: $45,000", status: "good", metadata: "+5% YTD" },
                { label: "Crypto: $25,250", status: "warning", metadata: "-8% YTD" },
                { label: "Cash: $20,000", metadata: "Emergency fund" },
              ],
            },
            {
              icon: <BarChart3 size={16} className="text-neutral-50" />,
              label: "Performance",
              hasDropdown: true,
              children: [
                { label: "1 Month: +2.5%", status: "good" },
                { label: "3 Months: +5.2%", status: "good" },
                { label: "6 Months: +8.7%", status: "good" },
                { label: "1 Year: +12.5%", status: "good" },
              ],
            },
          ],
        },
        {
          title: "Budgeting",
          items: [
            {
              icon: <PiggyBank size={16} className="text-neutral-50" />,
              label: "Budget Categories",
              hasDropdown: true,
              children: [
                { label: "Housing: $2,200 / $2,500", status: "good", metadata: "88% used" },
                { label: "Food: $780 / $800", status: "good", metadata: "97% used" },
                { label: "Transport: $450 / $600", status: "good", metadata: "75% used" },
                { label: "Entertainment: $320 / $400", status: "good", metadata: "80% used" },
                { label: "Savings: $2,000", status: "good", metadata: "Target met" },
              ],
            },
            {
              icon: <Target size={16} className="text-neutral-50" />,
              label: "Financial Goals",
              hasDropdown: true,
              children: [
                { label: "Emergency Fund", status: "good", metadata: "$20,000 / $20,000" },
                { label: "Home Down Payment", status: "warning", metadata: "$45,000 / $80,000" },
                { label: "Retirement", status: "good", metadata: "On track" },
              ],
            },
          ],
        },
        {
          title: "Quick Actions",
          items: [
            {
              icon: <Plus size={16} className="text-neutral-50" />,
              label: "Add transaction",
              shortcut: "⌘⇧T",
            },
            {
              icon: <Receipt size={16} className="text-neutral-50" />,
              label: "View all transactions",
            },
          ],
        },
      ],
    },

    // TASKS
    tasks: {
      title: "Tasks",
      headerBadge: { count: 12 },
      sections: [
        {
          title: "My Tasks",
          items: [
            {
              icon: <Time size={16} className="text-neutral-50" />,
              label: "Due today",
              badge: { count: 3, status: "warning" },
              hasDropdown: true,
              children: [
                { label: "Review design mockups", status: "warning" },
                { label: "Update documentation", status: "warning" },
                { label: "Test new feature", status: "warning" },
              ],
            },
            {
              icon: <InProgress size={16} className="text-neutral-50" />,
              label: "In progress",
              badge: { count: 5 },
              hasDropdown: true,
              children: [
                { label: "Implement user auth", metadata: "60% complete" },
                { label: "Database migration", metadata: "30% complete" },
              ],
            },
            {
              icon: <CheckmarkOutline size={16} className="text-neutral-50" />,
              label: "Completed",
              badge: { count: 8, status: "success" },
              hasDropdown: true,
              children: [
                { label: "Fixed login bug", metadata: "Today" },
                { label: "Updated dependencies", metadata: "Today" },
                { label: "Code review completed", metadata: "Yesterday" },
              ],
            },
          ],
        },
        {
          title: "Other",
          items: [
            {
              icon: <Flag size={16} className="text-neutral-50" />,
              label: "Priority tasks",
              badge: { count: 2, status: "error" },
              hasDropdown: true,
              children: [
                { label: "Security update", status: "error" },
                { label: "Client presentation", status: "warning" },
              ],
            },
            {
              icon: <Archive size={16} className="text-neutral-50" />,
              label: "Archived",
            },
          ],
        },
        {
          title: "Quick Actions",
          items: [
            {
              icon: <AddLarge size={16} className="text-neutral-50" />,
              label: "New task",
              shortcut: "⌘T",
            },
            {
              icon: <Filter size={16} className="text-neutral-50" />,
              label: "Filter tasks",
              shortcut: "⌘F",
            },
          ],
        },
      ],
    },

    // PROJECTS
    projects: {
      title: "Projects",
      headerBadge: { count: 3, status: "success" },
      sections: [
        {
          title: "Active Projects",
          items: [
            {
              icon: <FolderOpen size={16} className="text-neutral-50" />,
              label: "Web Application",
              status: "good",
              badge: { count: 8 },
              hasDropdown: true,
              children: [
                { label: "Frontend development", status: "good", metadata: "On track" },
                { label: "API integration", status: "warning", metadata: "Delayed" },
                { label: "Testing & QA", status: "good", metadata: "In progress" },
              ],
            },
            {
              icon: <FolderOpen size={16} className="text-neutral-50" />,
              label: "Mobile App",
              status: "good",
              badge: { count: 5 },
              hasDropdown: true,
              children: [
                { label: "UI/UX design", status: "good", metadata: "Complete" },
                { label: "Native development", status: "good", metadata: "70% done" },
              ],
            },
          ],
        },
        {
          title: "Other",
          items: [
            {
              icon: <CheckmarkOutline size={16} className="text-neutral-50" />,
              label: "Completed",
              badge: { count: 12, status: "success" },
            },
            {
              icon: <Archive size={16} className="text-neutral-50" />,
              label: "Archived",
            },
          ],
        },
        {
          title: "Quick Actions",
          items: [
            {
              icon: <AddLarge size={16} className="text-neutral-50" />,
              label: "New project",
              shortcut: "⌘⇧P",
            },
            {
              icon: <Filter size={16} className="text-neutral-50" />,
              label: "Filter projects",
            },
          ],
        },
      ],
    },

    // CALENDAR
    calendar: {
      title: "Calendar",
      headerBadge: { count: 7 },
      sections: [
        {
          title: "Views",
          items: [
            {
              icon: <View size={16} className="text-neutral-50" />,
              label: "Month view",
              shortcut: "⌘1",
            },
            {
              icon: <CalendarIcon size={16} className="text-neutral-50" />,
              label: "Week view",
              shortcut: "⌘2",
            },
            {
              icon: <Time size={16} className="text-neutral-50" />,
              label: "Day view",
              shortcut: "⌘3",
            },
          ],
        },
        {
          title: "Events",
          items: [
            {
              icon: <Time size={16} className="text-neutral-50" />,
              label: "Today's events",
              badge: { count: 4 },
              hasDropdown: true,
              children: [
                { label: "Team standup", metadata: "9:00 AM · 30m", status: "warning" },
                { label: "Client call", metadata: "2:00 PM · 1h" },
                { label: "Project review", metadata: "4:00 PM · 45m" },
                { label: "Soccer practice", metadata: "6:00 PM · Personal", status: "good" },
              ],
            },
            {
              icon: <CalendarIcon size={16} className="text-neutral-50" />,
              label: "Upcoming events",
              badge: { count: 12 },
            },
          ],
        },
        {
          title: "Quick Actions",
          items: [
            {
              icon: <AddLarge size={16} className="text-neutral-50" />,
              label: "New event",
              shortcut: "⌘E",
            },
            {
              icon: <Share size={16} className="text-neutral-50" />,
              label: "Share calendar",
            },
          ],
        },
      ],
    },

    // FILES
    files: {
      title: "Files",
      sections: [
        {
          title: "Recent Files",
          items: [
            {
              icon: <DocumentAdd size={16} className="text-neutral-50" />,
              label: "Recent documents",
              hasDropdown: true,
              children: [
                { label: "Project proposal.pdf", metadata: "Modified 2h ago" },
                { label: "Meeting notes.docx", metadata: "Modified today" },
                { label: "Design specs.figma", metadata: "Modified yesterday" },
              ],
            },
            {
              icon: <Share size={16} className="text-neutral-50" />,
              label: "Shared with me",
              badge: { count: 3 },
            },
            {
              icon: <Star size={16} className="text-neutral-50" />,
              label: "Starred files",
              badge: { count: 8 },
            },
          ],
        },
        {
          title: "Organization",
          items: [
            {
              icon: <Folder size={16} className="text-neutral-50" />,
              label: "All folders",
              hasDropdown: true,
              children: [
                { label: "Work Projects", metadata: "45 files" },
                { label: "Personal", metadata: "23 files" },
                { label: "Family Photos", metadata: "156 files" },
              ],
            },
            {
              icon: <Archive size={16} className="text-neutral-50" />,
              label: "Archived files",
            },
          ],
        },
        {
          title: "Quick Actions",
          items: [
            {
              icon: <CloudUpload size={16} className="text-neutral-50" />,
              label: "Upload file",
              shortcut: "⌘U",
            },
            {
              icon: <AddLarge size={16} className="text-neutral-50" />,
              label: "New folder",
            },
          ],
        },
      ],
    },

    // ANALYTICS HUB
    analytics: {
      title: "Analytics Hub",
      sections: [
        {
          title: "Performance Analytics",
          items: [
            {
              icon: <TrendingUp size={16} className="text-neutral-50" />,
              label: "Work Performance",
              hasDropdown: true,
              children: [
                { label: "Task completion rate: 87%", status: "good" },
                { label: "Productivity trend: +12%", status: "good" },
                { label: "Time allocation analysis" },
                { label: "Team efficiency metrics" },
              ],
            },
            {
              icon: <BarChart3 size={16} className="text-neutral-50" />,
              label: "Life Balance",
              hasDropdown: true,
              children: [
                { label: "Work hours: 42h/week", status: "good" },
                { label: "Family time: 18h/week", status: "good" },
                { label: "Personal time: 12h/week", status: "warning" },
                { label: "Sleep quality: 85%", status: "good" },
              ],
            },
          ],
        },
        {
          title: "Home & Energy",
          items: [
            {
              icon: <Zap size={16} className="text-neutral-50" />,
              label: "Energy Analytics",
              hasDropdown: true,
              children: [
                { label: "This month: 420 kWh", metadata: "-8% vs last month" },
                { label: "Cost: $54.60", status: "good" },
                { label: "Peak usage: 6-8 PM", metadata: "Optimization available" },
                { label: "Savings potential: $12/mo", status: "info" },
              ],
            },
            {
              icon: <Gauge size={16} className="text-neutral-50" />,
              label: "Resource Usage",
              hasDropdown: true,
              children: [
                { label: "Water: 3,200L", metadata: "Normal usage" },
                { label: "Gas: 85 m³", metadata: "Seasonal high" },
                { label: "Internet: 850 GB", metadata: "Heavy usage" },
              ],
            },
          ],
        },
        {
          title: "Insights",
          items: [
            {
              icon: <StarFilled size={16} className="text-neutral-50" />,
              label: "AI Predictions",
              badge: { count: 5, status: "info" },
              hasDropdown: true,
              children: [
                { label: "Task completion forecast", metadata: "95% on time" },
                { label: "Energy cost prediction", metadata: "$52 next month" },
                { label: "Productivity pattern", metadata: "Peak: 10 AM-12 PM" },
                { label: "Optimization suggestions", badge: { count: 5 } },
              ],
            },
          ],
        },
        {
          title: "Reports",
          items: [
            {
              icon: <Report size={16} className="text-neutral-50" />,
              label: "Weekly Reports",
            },
            {
              icon: <Report size={16} className="text-neutral-50" />,
              label: "Monthly Summary",
            },
          ],
        },
      ],
    },

    // SOCIAL MEDIA
    social: {
      title: "Social Media",
      sections: [
        {
          title: "Connected Accounts",
          items: [
            {
              icon: <Facebook size={16} className="text-neutral-50" />,
              label: "Facebook",
              status: "good",
              badge: { count: 12 },
            },
            {
              icon: <Instagram size={16} className="text-neutral-50" />,
              label: "Instagram",
              status: "good",
              badge: { count: 8 },
            },
            {
              icon: <Youtube size={16} className="text-neutral-50" />,
              label: "YouTube",
              status: "good",
              badge: { count: 3 },
            },
            {
              icon: <Twitter size={16} className="text-neutral-50" />,
              label: "X",
              status: "good",
              badge: { count: 24 },
            },
          ],
        },
        {
          title: "Quick Actions",
          items: [
            {
              icon: <Plus size={16} className="text-neutral-50" />,
              label: "Add social media account",
            },
          ],
        },
      ],
    },

    // NODES
    nodes: {
      title: "Compute Nodes",
      headerBadge: { status: "success" },
      sections: [
        {
          title: "Active Devices",
          items: [
            {
              icon: <Monitor size={16} className="text-neutral-50" />,
              label: "PC Workstation",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Compute: 85% available", status: "good" },
                { label: "Storage: 2.4 TB free", status: "good" },
                { label: "Memory: 32 GB available", status: "good" },
                { label: "GPU: NVIDIA RTX 4090", status: "good" },
              ],
            },
            {
              icon: <Laptop size={16} className="text-neutral-50" />,
              label: "Laptop",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Compute: 60% available", status: "good" },
                { label: "Storage: 512 GB free", status: "warning" },
                { label: "Memory: 16 GB available", status: "good" },
                { label: "Battery: 85%", status: "good" },
              ],
            },
            {
              icon: <Smartphone size={16} className="text-neutral-50" />,
              label: "Phone",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Compute: 40% available", status: "good" },
                { label: "Storage: 128 GB free", status: "good" },
                { label: "Memory: 8 GB available", status: "good" },
                { label: "Battery: 92%", status: "good" },
              ],
            },
            {
              icon: <Tablet size={16} className="text-neutral-50" />,
              label: "iPad / Tablet",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Compute: 55% available", status: "good" },
                { label: "Storage: 256 GB free", status: "good" },
                { label: "Memory: 12 GB available", status: "good" },
                { label: "Battery: 67%", status: "warning" },
              ],
            },
            {
              icon: <Server size={16} className="text-neutral-50" />,
              label: "Server",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Compute: 95% available", status: "good" },
                { label: "Storage: 10 TB free", status: "good" },
                { label: "Memory: 128 GB available", status: "good" },
                { label: "Uptime: 45 days", status: "good" },
              ],
            },
          ],
        },
        {
          title: "Hive Mind Network",
          items: [
            {
              icon: <Network size={16} className="text-neutral-50" />,
              label: "Distributed Inference",
              status: "good",
              badge: { count: 12, status: "success" },
              hasDropdown: true,
              children: [
                { label: "Active tasks: 12", status: "good" },
                { label: "Total throughput: 2.4 TF/s", status: "good" },
                { label: "Network latency: 45ms", status: "good" },
                { label: "Nodes synced: 5/5", status: "good" },
              ],
            },
            {
              icon: <Cpu size={16} className="text-neutral-50" />,
              label: "Compute Pool",
              hasDropdown: true,
              children: [
                { label: "Total CPU: 48 cores", metadata: "Utilization: 67%" },
                { label: "Total GPU: 3 devices", metadata: "AI workloads ready" },
                { label: "Performance: High", status: "good" },
              ],
            },
            {
              icon: <HardDrive size={16} className="text-neutral-50" />,
              label: "Storage Pool",
              hasDropdown: true,
              children: [
                { label: "Total capacity: 13.3 TB", status: "good" },
                { label: "Used: 4.2 TB (31%)", status: "good" },
                { label: "Redundancy: RAID 5", status: "good" },
                { label: "Health: Excellent", status: "good" },
              ],
            },
            {
              icon: <Activity size={16} className="text-neutral-50" />,
              label: "Network Status",
              status: "good",
            },
          ],
        },
        {
          title: "Quick Actions",
          items: [
            {
              icon: <Plus size={16} className="text-neutral-50" />,
              label: "Add new node",
            },
            {
              icon: <Gauge size={16} className="text-neutral-50" />,
              label: "Performance monitor",
            },
          ],
        },
      ],
    },

    // SMART HOME
    smarthome: {
      title: "Smart Home",
      headerBadge: { status: "success" },
      sections: [
        {
          title: "Scenes & Automation",
          items: [
            {
              icon: <Sparkles size={16} className="text-neutral-50" />,
              label: "Active Scenes",
              hasDropdown: true,
              children: [
                { label: "Morning Routine", status: "good", metadata: "Completed" },
                { label: "Work Mode", status: "good", metadata: "Active now" },
                { label: "Evening Relaxation", metadata: "Scheduled 6 PM" },
                { label: "Sleep Time", metadata: "Scheduled 10 PM" },
              ],
            },
            {
              icon: <Workflow size={16} className="text-neutral-50" />,
              label: "Automations",
              badge: { count: 8, status: "success" },
              hasDropdown: true,
              children: [
                { label: "Auto lights on motion", status: "good", metadata: "8 devices" },
                { label: "Climate optimization", status: "good", metadata: "Active" },
                { label: "Security arm/disarm", status: "good", metadata: "Location-based" },
                { label: "Energy management", status: "good", metadata: "AI powered" },
              ],
            },
          ],
        },
        {
          title: "Lighting Control",
          items: [
            {
              icon: <Lightbulb size={16} className="text-neutral-50" />,
              label: "All Lights",
              badge: { count: 12 },
              hasDropdown: true,
              children: [
                { label: "Living Room: On", status: "good", metadata: "100%" },
                { label: "Bedroom: Off", status: "offline" },
                { label: "Kitchen: On", status: "good", metadata: "80%" },
                { label: "Bathroom: Off", status: "offline" },
              ],
            },
          ],
        },
        {
          title: "Entertainment",
          items: [
            {
              icon: <Tv size={16} className="text-neutral-50" />,
              label: "TV & Media",
              hasDropdown: true,
              children: [
                { label: "Living Room TV: On", status: "good", metadata: "Netflix" },
                { label: "Bedroom TV: Off", status: "offline" },
                { label: "Home Theater: Standby", status: "away" },
              ],
            },
          ],
        },
        {
          title: "Climate Control",
          items: [
            {
              icon: <Thermometer size={16} className="text-neutral-50" />,
              label: "AC / Thermostat",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Temperature: 72°F", status: "good" },
                { label: "Mode: Auto", metadata: "AI optimized" },
                { label: "Fan speed: Medium", status: "good" },
                { label: "Schedule: Active", status: "good" },
              ],
            },
          ],
        },
        {
          title: "Utilities Monitoring",
          items: [
            {
              icon: <Zap size={16} className="text-neutral-50" />,
              label: "Energy",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Current usage: 2.4 kW", status: "good" },
                { label: "Today: 18.5 kWh", status: "good" },
                { label: "This month: 420 kWh", metadata: "-8% vs last" },
                { label: "Cost estimate: $54.60", status: "good" },
              ],
            },
            {
              icon: <Flame size={16} className="text-neutral-50" />,
              label: "Gas",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Current flow: 0.8 m³/h", status: "good" },
                { label: "Today: 4.2 m³", status: "good" },
                { label: "This month: 85 m³", status: "good" },
                { label: "Cost estimate: $38.25", status: "good" },
              ],
            },
            {
              icon: <Droplets size={16} className="text-neutral-50" />,
              label: "Water",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Current flow: 2.5 L/min", status: "good" },
                { label: "Today: 180 L", status: "good" },
                { label: "This month: 3,200 L", status: "good" },
                { label: "Cost estimate: $12.80", status: "good" },
              ],
            },
          ],
        },
        {
          title: "Quick Actions",
          items: [
            {
              icon: <ScanSearch size={16} className="text-neutral-50" />,
              label: "Add new device (Auto-detect)",
            },
          ],
        },
      ],
    },

    // SECURITY
    security: {
      title: "Security",
      headerBadge: { status: "success" },
      sections: [
        {
          title: "Threat Monitoring",
          items: [
            {
              icon: <ShieldAlert size={16} className="text-neutral-50" />,
              label: "Security Status",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Overall status: Secure", status: "good" },
                { label: "Active alerts: 0", status: "good" },
                { label: "Last scan: 15m ago", status: "good" },
                { label: "Threat level: Low", status: "good" },
              ],
            },
            {
              icon: <Activity size={16} className="text-neutral-50" />,
              label: "Recent Activity",
              badge: { count: 3 },
              hasDropdown: true,
              children: [
                { label: "Front door unlocked", metadata: "5m ago", status: "good" },
                { label: "Garage opened", metadata: "2h ago", status: "good" },
                { label: "Motion detected: Backyard", metadata: "3h ago", status: "good" },
              ],
            },
          ],
        },
        {
          title: "Video Surveillance",
          items: [
            {
              icon: <Camera size={16} className="text-neutral-50" />,
              label: "Cameras",
              badge: { count: 4, status: "success" },
              hasDropdown: true,
              children: [
                { label: "Front Door: Active", status: "good", metadata: "Recording" },
                { label: "Backyard: Active", status: "good", metadata: "Recording" },
                { label: "Garage: Active", status: "good", metadata: "Recording" },
                { label: "Driveway: Active", status: "good", metadata: "Recording" },
              ],
            },
          ],
        },
        {
          title: "Home Security",
          items: [
            {
              icon: <Lock size={16} className="text-neutral-50" />,
              label: "Locks & Access",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Front door: Locked", status: "good" },
                { label: "Back door: Locked", status: "good" },
                { label: "Garage: Unlocked", status: "warning" },
                { label: "Smart lock battery: 78%", status: "good" },
              ],
            },
            {
              icon: <ShieldAlert size={16} className="text-neutral-50" />,
              label: "Alarm System",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Status: Armed (Home)", status: "good" },
                { label: "Motion sensors: 6 active", status: "good" },
                { label: "Door sensors: 4 active", status: "good" },
                { label: "Window sensors: 8 active", status: "good" },
              ],
            },
          ],
        },
        {
          title: "Network Security",
          items: [
            {
              icon: <Wifi size={16} className="text-neutral-50" />,
              label: "Network Monitor",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Devices connected: 18", status: "good" },
                { label: "Threats blocked: 0", status: "good" },
                { label: "Firewall: Active", status: "good" },
                { label: "VPN: Connected", status: "good" },
              ],
            },
            {
              icon: <Shield size={16} className="text-neutral-50" />,
              label: "Protection Status",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "System health: Excellent", status: "good" },
                { label: "Last update: Today", status: "good" },
                { label: "All systems current", status: "good" },
              ],
            },
          ],
        },
        {
          title: "Family Safety",
          items: [
            {
              icon: <MapPin size={16} className="text-neutral-50" />,
              label: "Family Location",
              status: "good",
              hasDropdown: true,
              children: [
                { label: "Partner: At home", status: "good", metadata: "Safe zone" },
                { label: "Kids: At school", status: "good", metadata: "Safe zone" },
                { label: "Safe zones: 3 active", status: "good" },
                { label: "Alerts: None", status: "good" },
              ],
            },
          ],
        },
      ],
    },

    // SETTINGS
    settings: {
      title: "Settings",
      sections: [
        {
          title: "Account",
          items: [
            {
              icon: <User size={16} className="text-neutral-50" />,
              label: "Profile settings",
            },
            {
              icon: <Security size={16} className="text-neutral-50" />,
              label: "Security & Privacy",
            },
            {
              icon: <Notification size={16} className="text-neutral-50" />,
              label: "Notifications",
              badge: { count: 8 },
            },
          ],
        },
        {
          title: "Workspace",
          items: [
            {
              icon: <Settings size={16} className="text-neutral-50" />,
              label: "Preferences",
              hasDropdown: true,
              children: [
                { label: "Theme settings" },
                { label: "Time zone" },
                { label: "Default notifications" },
                { label: "Keyboard shortcuts" },
              ],
            },
            {
              icon: <Integration size={16} className="text-neutral-50" />,
              label: "Integrations",
              badge: { count: 12, status: "success" },
            },
          ],
        },
        {
          title: "AI & Automation",
          items: [
            {
              icon: <Bot size={16} className="text-neutral-50" />,
              label: "AI Settings",
              hasDropdown: true,
              children: [
                { label: "Conversation preferences" },
                { label: "Multi-modal settings" },
                { label: "Privacy controls" },
                { label: "Training data" },
              ],
            },
            {
              icon: <Workflow size={16} className="text-neutral-50" />,
              label: "Automation Rules",
              badge: { count: 23, status: "success" },
            },
          ],
        },
      ],
    },
  };

  return contentMap[activeSection] || contentMap.dashboard;
}

function IconNavButton({
  children,
  isActive = false,
  onClick,
  badge,
  workspace,
}: {
  children: React.ReactNode;
  isActive?: boolean;
  onClick?: () => void;
  badge?: NotificationBadge;
  workspace?: WorkspaceType;
}) {
  const workspaceColors: Record<WorkspaceType, string> = {
    all: "",
    productivity: "hover:border-blue-500/30",
    communication: "hover:border-green-500/30",
    intelligence: "hover:border-purple-500/30",
    infrastructure: "hover:border-orange-500/30",
    life: "hover:border-pink-500/30",
    business: "hover:border-yellow-500/30",
  };

  return (
    <div
      className={`box-border content-stretch flex flex-row items-center justify-center p-0 relative rounded-lg shrink-0 size-10 min-w-10 cursor-pointer transition-all duration-500 ${
        isActive
          ? "bg-neutral-800 text-neutral-50"
          : "hover:bg-neutral-900 text-neutral-400 hover:text-neutral-300"
      } ${workspace ? workspaceColors[workspace] : ""}`}
      style={{ transitionTimingFunction: softSpringEasing }}
      data-name="Icon Nav Button"
      onClick={onClick}
    >
      {children}
      {badge && <Badge {...badge} />}
    </div>
  );
}

// Workspace Switcher Component
function WorkspaceSwitcher({
  activeWorkspace,
  onWorkspaceChange,
}: {
  activeWorkspace: WorkspaceType;
  onWorkspaceChange: (workspace: WorkspaceType) => void;
}) {
  const [isOpen, setIsOpen] = useState(false);

  const workspaces: { id: WorkspaceType; label: string; icon: React.ReactNode; color: string }[] = [
    { id: "all", label: "All", icon: <Layers size={14} />, color: "text-neutral-50" },
    { id: "productivity", label: "Work", icon: <Briefcase size={14} />, color: "text-blue-400" },
    { id: "communication", label: "People", icon: <Users size={14} />, color: "text-green-400" },
    { id: "intelligence", label: "AI", icon: <Bot size={14} />, color: "text-purple-400" },
    { id: "infrastructure", label: "Tech", icon: <Network size={14} />, color: "text-orange-400" },
    { id: "life", label: "Life", icon: <Heart size={14} />, color: "text-pink-400" },
    { id: "business", label: "Business", icon: <Scale size={14} />, color: "text-yellow-400" },
  ];

  const activeWs = workspaces.find(w => w.id === activeWorkspace) || workspaces[0];

  return (
    <div className="relative mb-4">
      <div
        className="bg-neutral-900 rounded-lg size-10 flex items-center justify-center cursor-pointer hover:bg-neutral-800 transition-colors relative"
        onClick={() => setIsOpen(!isOpen)}
      >
        <div className={activeWs.color}>{activeWs.icon}</div>
      </div>
      {isOpen && (
        <>
          <div
            className="fixed inset-0 z-10"
            onClick={() => setIsOpen(false)}
          />
          <div className="absolute left-14 top-0 bg-neutral-900 rounded-lg p-2 min-w-[140px] z-20 border border-neutral-800 shadow-xl">
            <div className="text-[10px] text-neutral-500 px-2 py-1 uppercase tracking-wider">
              Workspace
            </div>
            {workspaces.map((ws) => (
              <div
                key={ws.id}
                className={`flex items-center gap-2 px-2 py-1.5 rounded cursor-pointer transition-colors ${
                  activeWorkspace === ws.id
                    ? "bg-neutral-800"
                    : "hover:bg-neutral-800"
                }`}
                onClick={() => {
                  onWorkspaceChange(ws.id);
                  setIsOpen(false);
                }}
              >
                <div className={ws.color}>{ws.icon}</div>
                <span className="text-[13px] text-neutral-50">
                  {ws.label}
                </span>
              </div>
            ))}
          </div>
        </>
      )}
    </div>
  );
}

// Quick Switcher Modal
function QuickSwitcher({
  isOpen,
  onClose,
  onNavigate,
}: {
  isOpen: boolean;
  onClose: () => void;
  onNavigate: (section: string) => void;
}) {
  const [searchQuery, setSearchQuery] = useState("");

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if ((e.metaKey || e.ctrlKey) && e.key === "k") {
        e.preventDefault();
        if (isOpen) {
          onClose();
        }
      }
      if (e.key === "Escape" && isOpen) {
        onClose();
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    return () => window.removeEventListener("keydown", handleKeyDown);
  }, [isOpen, onClose]);

  if (!isOpen) return null;

  const allSections = [
    { id: "ai", label: "AI Command Center", icon: <Bot size={16} />, category: "Intelligence" },
    { id: "dashboard", label: "Life Dashboard", icon: <Dashboard size={16} />, category: "Overview" },
    { id: "contacts", label: "Contacts", icon: <Users size={16} />, category: "Communication" },
    { id: "wallet", label: "Wallet", icon: <Wallet size={16} />, category: "Finance" },
    { id: "legal", label: "Legal & Compliance", icon: <Scale size={16} />, category: "Business" },
    { id: "tasks", label: "Tasks", icon: <Task size={16} />, category: "Productivity" },
    { id: "projects", label: "Projects", icon: <Folder size={16} />, category: "Productivity" },
    { id: "calendar", label: "Calendar", icon: <Calendar size={16} />, category: "Productivity" },
    { id: "files", label: "Files", icon: <DocumentAdd size={16} />, category: "Productivity" },
    { id: "analytics", label: "Analytics Hub", icon: <Analytics size={16} />, category: "Insights" },
    { id: "social", label: "Social Media", icon: <Share2 size={16} />, category: "Communication" },
    { id: "nodes", label: "Compute Nodes", icon: <Network size={16} />, category: "Infrastructure" },
    { id: "smarthome", label: "Smart Home", icon: <Home size={16} />, category: "Life" },
    { id: "security", label: "Security", icon: <ShieldAlert size={16} />, category: "Life" },
  ];

  const filteredSections = allSections.filter((section) =>
    section.label.toLowerCase().includes(searchQuery.toLowerCase()) ||
    section.category.toLowerCase().includes(searchQuery.toLowerCase())
  );

  return (
    <div className="fixed inset-0 bg-black/50 backdrop-blur-sm flex items-start justify-center pt-[20vh] z-50">
      <div className="bg-neutral-900 rounded-xl border border-neutral-800 w-[600px] max-h-[400px] overflow-hidden shadow-2xl">
        <div className="p-4 border-b border-neutral-800">
          <div className="flex items-center gap-3 bg-neutral-950 rounded-lg px-4 py-3">
            <Search size={18} className="text-neutral-400" />
            <input
              type="text"
              placeholder="Search sections, features, shortcuts..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              className="flex-1 bg-transparent border-none outline-none text-neutral-50 placeholder:text-neutral-500"
              autoFocus
            />
            <kbd className="px-2 py-1 bg-neutral-900 rounded text-[11px] text-neutral-400 border border-neutral-800">
              ESC
            </kbd>
          </div>
        </div>
        <div className="overflow-y-auto max-h-[300px] p-2">
          {filteredSections.map((section) => (
            <div
              key={section.id}
              className="flex items-center gap-3 px-4 py-3 rounded-lg hover:bg-neutral-800 cursor-pointer transition-colors"
              onClick={() => {
                onNavigate(section.id);
                onClose();
                setSearchQuery("");
              }}
            >
              <div className="text-neutral-50">{section.icon}</div>
              <div className="flex-1">
                <div className="text-[14px] text-neutral-50">{section.label}</div>
                <div className="text-[12px] text-neutral-500">{section.category}</div>
              </div>
              <ChevronRight size={16} className="text-neutral-600" />
            </div>
          ))}
          {filteredSections.length === 0 && (
            <div className="text-center py-8 text-neutral-500">
              No results found for "{searchQuery}"
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

function IconNavigation({
  activeSection,
  onSectionChange,
  activeWorkspace,
  onWorkspaceChange,
  onNotificationsClick,
  onFavoritesClick,
}: {
  activeSection: string;
  onSectionChange: (section: string) => void;
  activeWorkspace: WorkspaceType;
  onWorkspaceChange: (workspace: WorkspaceType) => void;
  onNotificationsClick: () => void;
  onFavoritesClick: () => void;
}) {
  const navItems = [
    {
      id: "ai",
      icon: <Bot size={16} />,
      label: "AI Command Center",
      badge: { count: 3, status: "info" as const },
      workspace: "intelligence" as WorkspaceType,
      shortcut: "⌘1",
    },
    {
      id: "dashboard",
      icon: <Dashboard size={16} />,
      label: "Life Dashboard",
      workspace: "all" as WorkspaceType,
      shortcut: "⌘D",
    },
    {
      id: "contacts",
      icon: <Users size={16} />,
      label: "Contacts",
      badge: { count: 7, status: "info" as const },
      workspace: "communication" as WorkspaceType,
      shortcut: "⌘P",
    },
    {
      id: "wallet",
      icon: <Wallet size={16} />,
      label: "Wallet",
      badge: { count: 5, status: "warning" as const },
      workspace: "business" as WorkspaceType,
      shortcut: "⌘W",
    },
    {
      id: "legal",
      icon: <Scale size={16} />,
      label: "Legal & Compliance",
      badge: { count: 3, status: "warning" as const },
      workspace: "business" as WorkspaceType,
    },
    {
      id: "tasks",
      icon: <Task size={16} />,
      label: "Tasks",
      badge: { count: 12 },
      workspace: "productivity" as WorkspaceType,
      shortcut: "⌘T",
    },
    {
      id: "projects",
      icon: <Folder size={16} />,
      label: "Projects",
      badge: { count: 3, status: "success" as const },
      workspace: "productivity" as WorkspaceType,
    },
    {
      id: "calendar",
      icon: <Calendar size={16} />,
      label: "Calendar",
      badge: { count: 7 },
      workspace: "productivity" as WorkspaceType,
    },
    {
      id: "files",
      icon: <DocumentAdd size={16} />,
      label: "Files",
      workspace: "productivity" as WorkspaceType,
    },
    {
      id: "analytics",
      icon: <Analytics size={16} />,
      label: "Analytics Hub",
      workspace: "all" as WorkspaceType,
    },
    {
      id: "social",
      icon: <Share2 size={16} />,
      label: "Social Media",
      badge: { count: 47 },
      workspace: "communication" as WorkspaceType,
    },
    {
      id: "nodes",
      icon: <Network size={16} />,
      label: "Compute Nodes",
      badge: { status: "success" as const },
      workspace: "infrastructure" as WorkspaceType,
    },
    {
      id: "smarthome",
      icon: <Home size={16} />,
      label: "Smart Home",
      badge: { status: "success" as const },
      workspace: "life" as WorkspaceType,
    },
    {
      id: "security",
      icon: <ShieldAlert size={16} />,
      label: "Security",
      badge: { status: "success" as const },
      workspace: "life" as WorkspaceType,
    },
  ];

  const filteredNavItems =
    activeWorkspace === "all"
      ? navItems
      : navItems.filter((item) => item.workspace === activeWorkspace || item.workspace === "all");

  return (
    <div
      className="bg-[#000000] box-border content-stretch flex flex-col gap-2 h-[800px] items-center justify-start p-4 relative rounded-l-2xl shrink-0 w-16 border-r border-neutral-800"
      data-name="Icon Navigation"
    >
      {/* Workspace Switcher */}
      <WorkspaceSwitcher
        activeWorkspace={activeWorkspace}
        onWorkspaceChange={onWorkspaceChange}
      />

      {/* Logo */}
      <div className="mb-2 size-10 flex items-center justify-center">
        <div className="size-7">
          <InterfacesLogo1 />
        </div>
      </div>

      {/* Navigation Icons */}
      <div className="flex flex-col gap-2 w-full items-center flex-1 overflow-y-auto">
        {filteredNavItems.map((item) => (
          <IconNavButton
            key={item.id}
            isActive={activeSection === item.id}
            onClick={() => onSectionChange(item.id)}
            badge={item.badge}
            workspace={item.workspace}
          >
            {item.icon}
          </IconNavButton>
        ))}
      </div>

      {/* Bottom section - Notifications & Settings */}
      <div className="flex flex-col gap-2 w-full items-center pt-2 border-t border-neutral-800">
        <IconNavButton 
          badge={{ count: 6, pulse: true }}
          onClick={onNotificationsClick}
        >
          <Bell size={16} />
        </IconNavButton>
        <IconNavButton 
          badge={{ count: 5, status: "info" }}
          onClick={onFavoritesClick}
        >
          <Star size={16} />
        </IconNavButton>
        <IconNavButton
          isActive={activeSection === "settings"}
          onClick={() => onSectionChange("settings")}
        >
          <Settings size={16} />
        </IconNavButton>
        <div className="size-8">
          <Avatar />
        </div>
      </div>
    </div>
  );
}

function SectionTitle({
  title,
  onToggleCollapse,
  isCollapsed,
  badge,
}: {
  title: string;
  onToggleCollapse: () => void;
  isCollapsed: boolean;
  badge?: NotificationBadge;
}) {
  if (isCollapsed) {
    return (
      <div
        className="relative shrink-0 w-full flex justify-center transition-all duration-500"
        style={{ transitionTimingFunction: softSpringEasing }}
        data-name="Section Title Collapsed"
      >
        <button
          onClick={onToggleCollapse}
          className="box-border content-stretch flex flex-row items-center justify-center overflow-clip p-0 relative rounded-lg shrink-0 cursor-pointer transition-all duration-500 hover:bg-neutral-900 text-neutral-400 hover:text-neutral-300 size-10 min-w-10"
          style={{ transitionTimingFunction: softSpringEasing }}
        >
          <ChevronLeft
            size={16}
            className="transition-transform duration-500"
            style={{
              transitionTimingFunction: softSpringEasing,
              transform: "rotate(180deg)",
            }}
          />
        </button>
      </div>
    );
  }

  return (
    <div
      className="relative shrink-0 w-full overflow-hidden transition-all duration-500"
      style={{ transitionTimingFunction: softSpringEasing }}
      data-name="Section Title"
    >
      <div className="flex flex-row items-center justify-between relative size-full">
        <div
          className="box-border content-stretch flex flex-row items-center justify-start relative h-12 overflow-hidden transition-opacity opacity-100 duration-500"
          style={{ transitionTimingFunction: softSpringEasing }}
        >
          <div className="box-border content-stretch flex flex-col gap-2 items-start justify-center px-2 py-1 relative shrink-0">
            <div className="flex items-center gap-2">
              <div className="font-['Lexend:SemiBold',_sans-serif] font-semibold leading-[0] relative shrink-0 text-[18px] text-left text-neutral-50 text-nowrap">
                <p className="block leading-[27px] whitespace-pre">
                  {title}
                </p>
              </div>
              {badge && <Badge {...badge} />}
            </div>
          </div>
        </div>
        <div className="flex items-center justify-center pr-1">
          <button
            onClick={onToggleCollapse}
            className="box-border content-stretch flex flex-row items-center justify-center overflow-clip p-0 relative rounded-lg shrink-0 cursor-pointer transition-all duration-500 hover:bg-neutral-900 text-neutral-400 hover:text-neutral-300 size-10 min-w-10"
            style={{
              transitionTimingFunction: softSpringEasing,
            }}
          >
            <ChevronLeft
              size={16}
              className="transition-transform duration-500"
              style={{
                transitionTimingFunction: softSpringEasing,
              }}
            />
          </button>
        </div>
      </div>
    </div>
  );
}

function DetailSidebar({
  activeSection,
  onQuickSwitcher,
}: {
  activeSection: string;
  onQuickSwitcher: () => void;
}) {
  const [expandedItems, setExpandedItems] = useState<Set<string>>(
    new Set()
  );
  const [isCollapsed, setIsCollapsed] = useState(false);
  const content = getSidebarContent(activeSection);

  const toggleExpanded = (itemKey: string) => {
    const newExpanded = new Set(expandedItems);
    if (newExpanded.has(itemKey)) {
      newExpanded.delete(itemKey);
    } else {
      newExpanded.add(itemKey);
    }
    setExpandedItems(newExpanded);
  };

  const toggleCollapse = () => {
    setIsCollapsed(!isCollapsed);
  };

  return (
    <div
      className={`bg-[#000000] box-border content-stretch flex flex-col gap-5 h-[800px] items-start justify-start overflow-visible p-4 relative rounded-r-2xl shrink-0 transition-all duration-500 ${
        isCollapsed
          ? "w-16 min-w-16 !px-0 justify-center"
          : "w-80"
      }`}
      style={{ transitionTimingFunction: softSpringEasing }}
      data-name="Detail Sidebar"
    >
      <SectionTitle
        title={content.title}
        onToggleCollapse={toggleCollapse}
        isCollapsed={isCollapsed}
        badge={content.headerBadge}
      />
      <SearchContainer isCollapsed={isCollapsed} onQuickSwitcher={onQuickSwitcher} />

      <div
        className={`basis-0 box-border content-stretch flex flex-col grow min-h-px min-w-10 p-0 relative shrink-0 w-full overflow-y-auto transition-all duration-500 ${
          isCollapsed
            ? "gap-2 items-center justify-start"
            : "gap-6 items-start justify-start"
        }`}
        style={{ transitionTimingFunction: softSpringEasing }}
      >
        {content.sections.map((section, index) => (
          <MenuSection
            key={`${activeSection}-${index}`}
            section={section}
            expandedItems={expandedItems}
            onToggleExpanded={toggleExpanded}
            isCollapsed={isCollapsed}
          />
        ))}
      </div>
    </div>
  );
}

function TwoLevelSidebar() {
  const [activeSection, setActiveSection] = useState("ai");
  const [activeWorkspace, setActiveWorkspace] = useState<WorkspaceType>("all");
  const [quickSwitcherOpen, setQuickSwitcherOpen] = useState(false);
  const [notificationsOpen, setNotificationsOpen] = useState(false);
  const [favoritesOpen, setFavoritesOpen] = useState(false);

  // Keyboard shortcuts
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if ((e.metaKey || e.ctrlKey) && e.key === "k") {
        e.preventDefault();
        setQuickSwitcherOpen(true);
      }
      
      // Section shortcuts
      if ((e.metaKey || e.ctrlKey)) {
        const shortcuts: Record<string, string> = {
          "1": "ai",
          "d": "dashboard",
          "p": "contacts",
          "w": "wallet",
          "t": "tasks",
        };
        
        if (shortcuts[e.key]) {
          e.preventDefault();
          setActiveSection(shortcuts[e.key]);
        }
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    return () => window.removeEventListener("keydown", handleKeyDown);
  }, []);

  return (
    <>
      <div className="flex flex-row" data-name="Two Level Sidebar">
        <IconNavigation
          activeSection={activeSection}
          onSectionChange={setActiveSection}
          activeWorkspace={activeWorkspace}
          onWorkspaceChange={setActiveWorkspace}
          onNotificationsClick={() => setNotificationsOpen(true)}
          onFavoritesClick={() => setFavoritesOpen(true)}
        />
        <DetailSidebar 
          activeSection={activeSection}
          onQuickSwitcher={() => setQuickSwitcherOpen(true)}
        />
      </div>
      <QuickSwitcher
        isOpen={quickSwitcherOpen}
        onClose={() => setQuickSwitcherOpen(false)}
        onNavigate={setActiveSection}
      />
      <NotificationsPanel
        isOpen={notificationsOpen}
        onClose={() => setNotificationsOpen(false)}
      />
      <FavoritesPanel
        isOpen={favoritesOpen}
        onClose={() => setFavoritesOpen(false)}
        onNavigate={setActiveSection}
      />
    </>
  );
}

export function Frame760() {
  return (
    <div className="bg-[#1a1a1a] box-border content-stretch flex flex-row gap-4 items-center justify-center p-0 relative size-full min-h-screen">
      <TwoLevelSidebar />
    </div>
  );
}
