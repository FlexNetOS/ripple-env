import React, { useState } from 'react';
import { View, Text, Pressable, StyleSheet, ScrollView, Modal } from 'react-native';
import { useSidebar } from '@/contexts/sidebar-context';
import MaterialIcons from '@expo/vector-icons/MaterialIcons';

interface NotificationItem {
  id: string;
  title: string;
  message: string;
  time: string;
  type: 'alert' | 'update' | 'mention' | 'system';
  read: boolean;
  iconName: keyof typeof MaterialIcons.glyphMap;
  iconColor: string;
}

const defaultNotifications: NotificationItem[] = [
  {
    id: '1',
    title: 'Build Complete',
    message: 'P2 Agent Framework build finished successfully',
    time: '2m ago',
    type: 'update',
    read: false,
    iconName: 'check-circle',
    iconColor: '#4ade80',
  },
  {
    id: '2',
    title: 'High CPU Usage',
    message: 'Inference node at 94% CPU utilization',
    time: '5m ago',
    type: 'alert',
    read: false,
    iconName: 'warning',
    iconColor: '#facc15',
  },
  {
    id: '3',
    title: 'Agent Deployed',
    message: 'Validator Agent successfully deployed',
    time: '15m ago',
    type: 'update',
    read: false,
    iconName: 'rocket-launch',
    iconColor: '#60a5fa',
  },
  {
    id: '4',
    title: 'Test Suite Passed',
    message: '26 tests passed, 0 failed',
    time: '30m ago',
    type: 'update',
    read: true,
    iconName: 'science',
    iconColor: '#4ade80',
  },
  {
    id: '5',
    title: 'New Peer Connected',
    message: 'Node peer-7a3f joined the network',
    time: '1h ago',
    type: 'system',
    read: true,
    iconName: 'groups',
    iconColor: '#a78bfa',
  },
  {
    id: '6',
    title: 'Storage Warning',
    message: 'Vector DB approaching capacity limit',
    time: '2h ago',
    type: 'alert',
    read: true,
    iconName: 'storage',
    iconColor: '#f59e0b',
  },
];

export function NotificationsPanel() {
  const { showNotifications, setShowNotifications } = useSidebar();
  const [notifications, setNotifications] = useState<NotificationItem[]>(defaultNotifications);

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

  return (
    <Modal
      visible={showNotifications}
      transparent
      animationType="fade"
      onRequestClose={() => setShowNotifications(false)}
    >
      <Pressable 
        style={styles.overlay}
        onPress={() => setShowNotifications(false)}
      >
        <Pressable 
          style={styles.panel}
          onPress={(e) => e.stopPropagation()}
        >
          {/* Header */}
          <View style={styles.header}>
            <View style={styles.headerTitle}>
              <MaterialIcons name="notifications" size={20} color="#FAFAFA" />
              <Text style={styles.title}>Notifications</Text>
              {unreadCount > 0 && (
                <View style={styles.unreadBadge}>
                  <Text style={styles.unreadText}>{unreadCount}</Text>
                </View>
              )}
            </View>
            <Pressable
              onPress={() => setShowNotifications(false)}
              style={({ pressed }) => [
                styles.closeButton,
                pressed && { opacity: 0.7 },
              ]}
            >
              <MaterialIcons name="close" size={18} color="#9CA3AF" />
            </Pressable>
          </View>

          {unreadCount > 0 && (
            <Pressable onPress={markAllAsRead}>
              <Text style={styles.markAllRead}>Mark all as read</Text>
            </Pressable>
          )}

          {/* Notifications List */}
          <ScrollView style={styles.list} showsVerticalScrollIndicator={false}>
            {notifications.length === 0 ? (
              <View style={styles.emptyState}>
                <MaterialIcons name="notifications-none" size={48} color="#4B5563" />
                <Text style={styles.emptyText}>No notifications</Text>
              </View>
            ) : (
              notifications.map((notification) => (
                <Pressable
                  key={notification.id}
                  style={({ pressed }) => [
                    styles.notificationItem,
                    notification.read ? styles.notificationRead : styles.notificationUnread,
                    pressed && { opacity: 0.8 },
                  ]}
                  onPress={() => markAsRead(notification.id)}
                >
                  <View style={styles.notificationContent}>
                    <View style={[styles.iconContainer, { backgroundColor: `${notification.iconColor}20` }]}>
                      <MaterialIcons 
                        name={notification.iconName} 
                        size={16} 
                        color={notification.iconColor} 
                      />
                    </View>
                    <View style={styles.textContent}>
                      <Text style={styles.notificationTitle}>{notification.title}</Text>
                      <Text style={styles.notificationMessage} numberOfLines={2}>
                        {notification.message}
                      </Text>
                      <Text style={styles.notificationTime}>{notification.time}</Text>
                    </View>
                    <Pressable
                      onPress={() => deleteNotification(notification.id)}
                      style={({ pressed }) => [
                        styles.deleteButton,
                        pressed && { opacity: 0.7 },
                      ]}
                    >
                      <MaterialIcons name="close" size={14} color="#6B7280" />
                    </Pressable>
                  </View>
                </Pressable>
              ))
            )}
          </ScrollView>
        </Pressable>
      </Pressable>
    </Modal>
  );
}

const styles = StyleSheet.create({
  overlay: {
    flex: 1,
    backgroundColor: 'rgba(0, 0, 0, 0.5)',
    justifyContent: 'flex-start',
    alignItems: 'flex-end',
    paddingTop: 60,
    paddingRight: 16,
  },
  panel: {
    width: 360,
    maxHeight: '80%',
    backgroundColor: '#171717',
    borderRadius: 16,
    borderWidth: 1,
    borderColor: '#262626',
    overflow: 'hidden',
  },
  header: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: 16,
    borderBottomWidth: 1,
    borderBottomColor: '#262626',
  },
  headerTitle: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  title: {
    fontSize: 16,
    fontWeight: '600',
    color: '#FAFAFA',
  },
  unreadBadge: {
    backgroundColor: '#ef4444',
    borderRadius: 10,
    paddingHorizontal: 8,
    paddingVertical: 2,
  },
  unreadText: {
    fontSize: 11,
    fontWeight: '600',
    color: '#FFFFFF',
  },
  closeButton: {
    padding: 6,
    borderRadius: 8,
  },
  markAllRead: {
    fontSize: 13,
    color: '#3b82f6',
    paddingHorizontal: 16,
    paddingBottom: 12,
  },
  list: {
    flex: 1,
    padding: 8,
  },
  emptyState: {
    alignItems: 'center',
    justifyContent: 'center',
    paddingVertical: 48,
  },
  emptyText: {
    fontSize: 14,
    color: '#6B7280',
    marginTop: 12,
  },
  notificationItem: {
    padding: 12,
    marginBottom: 8,
    borderRadius: 12,
  },
  notificationUnread: {
    backgroundColor: '#262626',
  },
  notificationRead: {
    backgroundColor: '#0a0a0a',
  },
  notificationContent: {
    flexDirection: 'row',
    gap: 12,
  },
  iconContainer: {
    width: 36,
    height: 36,
    borderRadius: 10,
    alignItems: 'center',
    justifyContent: 'center',
  },
  textContent: {
    flex: 1,
  },
  notificationTitle: {
    fontSize: 14,
    fontWeight: '600',
    color: '#FAFAFA',
    marginBottom: 2,
  },
  notificationMessage: {
    fontSize: 13,
    color: '#9CA3AF',
    lineHeight: 18,
  },
  notificationTime: {
    fontSize: 11,
    color: '#6B7280',
    marginTop: 4,
  },
  deleteButton: {
    padding: 4,
  },
});

export default NotificationsPanel;
