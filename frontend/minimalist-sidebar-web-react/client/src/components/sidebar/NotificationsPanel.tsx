import React, { useEffect, useState } from 'react';
import { useSidebar } from '@/contexts/SidebarContext';
import { trpc } from '@/lib/trpc';
import * as LucideIcons from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';
import { formatDistanceToNow } from 'date-fns';

// Map notification types to icons and colors
const typeConfig: Record<string, { iconName: string; iconColor: string }> = {
  info: { iconName: 'Info', iconColor: '#60a5fa' },
  success: { iconName: 'CheckCircle', iconColor: '#4ade80' },
  warning: { iconName: 'AlertTriangle', iconColor: '#facc15' },
  error: { iconName: 'AlertCircle', iconColor: '#f87171' },
  alert: { iconName: 'Bell', iconColor: '#f59e0b' },
  update: { iconName: 'RefreshCw', iconColor: '#60a5fa' },
  mention: { iconName: 'AtSign', iconColor: '#a78bfa' },
  system: { iconName: 'Settings', iconColor: '#9CA3AF' },
};

// Polling interval in milliseconds (30 seconds)
const POLLING_INTERVAL = 30000;

function DynamicIcon({ name, size = 16, color }: { name: string; size?: number; color?: string }) {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const IconComponent = (LucideIcons as any)[name] || LucideIcons.Circle;
  return <IconComponent size={size} color={color} />;
}

function formatTime(date: Date | string): string {
  try {
    const d = typeof date === 'string' ? new Date(date) : date;
    return formatDistanceToNow(d, { addSuffix: true });
  } catch {
    return 'recently';
  }
}

export function NotificationsPanel() {
  const { showNotifications, setShowNotifications } = useSidebar();
  const utils = trpc.useUtils();
  const [lastFetchTime, setLastFetchTime] = useState<Date>(new Date());

  // Fetch real notifications from tRPC with polling
  const { data: notifications = [], isLoading, refetch } = trpc.notifications.list.useQuery(
    { limit: 20 },
    { 
      enabled: showNotifications,
      // Enable automatic refetching when panel is open
      refetchInterval: showNotifications ? POLLING_INTERVAL : false,
      refetchIntervalInBackground: false,
    }
  );

  // Update last fetch time when data changes
  useEffect(() => {
    if (notifications.length > 0) {
      setLastFetchTime(new Date());
    }
  }, [notifications]);

  // Manual refresh function
  const handleRefresh = async () => {
    await refetch();
    setLastFetchTime(new Date());
  };

  // Mutations
  const markAsReadMutation = trpc.notifications.markAsRead.useMutation({
    onSuccess: () => {
      utils.notifications.list.invalidate();
    },
  });

  const markAllAsReadMutation = trpc.notifications.markAllAsRead.useMutation({
    onSuccess: () => {
      utils.notifications.list.invalidate();
    },
  });

  const deleteMutation = trpc.notifications.delete.useMutation({
    onSuccess: () => {
      utils.notifications.list.invalidate();
    },
  });

  const markAsRead = (id: number) => {
    markAsReadMutation.mutate({ id });
  };

  const markAllAsRead = () => {
    markAllAsReadMutation.mutate();
  };

  const deleteNotification = (id: number) => {
    deleteMutation.mutate({ id });
  };

  const unreadCount = notifications.filter(n => !n.isRead).length;

  return (
    <AnimatePresence>
      {showNotifications && (
        <>
          {/* Backdrop */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            className="fixed inset-0 bg-black/50 z-50"
            onClick={() => setShowNotifications(false)}
          />
          
          {/* Panel */}
          <motion.div
            initial={{ opacity: 0, x: 20, scale: 0.95 }}
            animate={{ opacity: 1, x: 0, scale: 1 }}
            exit={{ opacity: 0, x: 20, scale: 0.95 }}
            transition={{ duration: 0.2 }}
            className="fixed top-16 right-4 w-[360px] max-h-[80vh] bg-[#171717] rounded-2xl border border-[#262626] overflow-hidden z-50 shadow-2xl"
            onClick={(e) => e.stopPropagation()}
          >
            {/* Header */}
            <div className="flex items-center justify-between p-4 border-b border-[#262626]">
              <div className="flex items-center gap-2">
                <LucideIcons.Bell size={20} className="text-[#FAFAFA]" />
                <span className="text-base font-semibold text-[#FAFAFA]">Notifications</span>
                {unreadCount > 0 && (
                  <span className="bg-red-500 text-white text-[11px] font-semibold px-2 py-0.5 rounded-full">
                    {unreadCount}
                  </span>
                )}
              </div>
              <div className="flex items-center gap-1">
                {/* Refresh button */}
                <button
                  onClick={handleRefresh}
                  className="p-1.5 rounded-lg hover:bg-[#262626] transition-colors group"
                  title={`Last updated: ${formatTime(lastFetchTime)}`}
                >
                  <LucideIcons.RefreshCw 
                    size={16} 
                    className={cn(
                      "text-[#9CA3AF] group-hover:text-[#00D4FF] transition-colors",
                      isLoading && "animate-spin"
                    )} 
                  />
                </button>
                <button
                  onClick={() => setShowNotifications(false)}
                  className="p-1.5 rounded-lg hover:bg-[#262626] transition-colors"
                >
                  <LucideIcons.X size={18} className="text-[#9CA3AF]" />
                </button>
              </div>
            </div>

            {/* Auto-refresh indicator */}
            <div className="px-4 py-2 bg-[#0a0a0a] border-b border-[#262626] flex items-center justify-between">
              <span className="text-[11px] text-[#6B7280]">
                Auto-refresh every 30s
              </span>
              <span className="text-[11px] text-[#4B5563]">
                Updated {formatTime(lastFetchTime)}
              </span>
            </div>

            {unreadCount > 0 && (
              <button 
                onClick={markAllAsRead}
                disabled={markAllAsReadMutation.isPending}
                className="text-[13px] text-[#00D4FF] hover:text-[#00D4FF]/80 px-4 py-3 text-left transition-colors disabled:opacity-50 w-full border-b border-[#262626]"
              >
                {markAllAsReadMutation.isPending ? 'Marking...' : 'Mark all as read'}
              </button>
            )}

            {/* Notifications List */}
            <div className="flex-1 overflow-y-auto p-2 max-h-[calc(80vh-180px)] scrollbar-thin">
              {isLoading && notifications.length === 0 ? (
                <div className="flex flex-col items-center justify-center py-12">
                  <LucideIcons.Loader2 size={32} className="text-[#00D4FF] animate-spin" />
                  <span className="text-sm text-[#6B7280] mt-3">Loading notifications...</span>
                </div>
              ) : notifications.length === 0 ? (
                <div className="flex flex-col items-center justify-center py-12">
                  <LucideIcons.BellOff size={48} className="text-[#4B5563]" />
                  <span className="text-sm text-[#6B7280] mt-3">No notifications</span>
                  <span className="text-xs text-[#4B5563] mt-1">You're all caught up!</span>
                </div>
              ) : (
                notifications.map((notification) => {
                  const config = typeConfig[notification.type] || typeConfig.info;
                  return (
                    <motion.button
                      key={notification.id}
                      initial={{ opacity: 0, y: 10 }}
                      animate={{ opacity: 1, y: 0 }}
                      className={cn(
                        "w-full p-3 mb-2 rounded-xl text-left transition-colors",
                        notification.isRead ? "bg-[#0a0a0a]" : "bg-[#262626]"
                      )}
                      onClick={() => !notification.isRead && markAsRead(notification.id)}
                    >
                      <div className="flex gap-3">
                        <div 
                          className="w-9 h-9 rounded-lg flex items-center justify-center flex-shrink-0"
                          style={{ backgroundColor: `${config.iconColor}20` }}
                        >
                          <DynamicIcon 
                            name={config.iconName} 
                            size={16} 
                            color={config.iconColor} 
                          />
                        </div>
                        <div className="flex-1 min-w-0">
                          <p className="text-sm font-semibold text-[#FAFAFA] mb-0.5">
                            {notification.title}
                          </p>
                          <p className="text-[13px] text-[#9CA3AF] line-clamp-2">
                            {notification.content}
                          </p>
                          <p className="text-[11px] text-[#6B7280] mt-1">
                            {formatTime(notification.createdAt)}
                          </p>
                        </div>
                        <button
                          onClick={(e) => {
                            e.stopPropagation();
                            deleteNotification(notification.id);
                          }}
                          disabled={deleteMutation.isPending}
                          className="p-1 hover:bg-[#333] rounded transition-colors flex-shrink-0 disabled:opacity-50"
                        >
                          <LucideIcons.X size={14} className="text-[#6B7280]" />
                        </button>
                      </div>
                    </motion.button>
                  );
                })
              )}
            </div>

            {/* Footer */}
            {notifications.length > 0 && (
              <div className="border-t border-[#262626] p-3">
                <a 
                  href="/notifications"
                  className="block text-center text-[13px] text-[#00D4FF] hover:text-[#00D4FF]/80 transition-colors"
                  onClick={() => setShowNotifications(false)}
                >
                  View all notifications
                </a>
              </div>
            )}
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}

export default NotificationsPanel;
