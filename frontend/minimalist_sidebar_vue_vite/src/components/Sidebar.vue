<script setup lang="ts">
import { computed } from 'vue';
import { useSidebar } from '@/composables/useSidebar';
import { useVibeKanban } from '@/composables/useVibeKanban';

const { state, navItems, activeContent, setActiveNavItem, toggleExpanded } = useSidebar();
const { isConnected: kanbanConnected, openDashboard } = useVibeKanban();

const activeNavItem = computed(() => state.value.activeNavItem);
const isExpanded = computed(() => state.value.isExpanded);

const handleNavClick = (itemId: string) => {
  if (itemId === 'kanban') {
    openDashboard();
  } else {
    setActiveNavItem(itemId);
  }
};

const getStatusColor = (status: string | undefined) => {
  const colors: Record<string, string> = {
    online: 'bg-green-500',
    good: 'bg-green-500',
    busy: 'bg-yellow-500',
    warning: 'bg-yellow-500',
    error: 'bg-red-500',
    offline: 'bg-gray-500',
  };
  return colors[status || ''] || 'bg-gray-400';
};
</script>

<template>
  <aside class="flex h-screen bg-sidebar text-sidebar-foreground">
    <!-- Icon Rail -->
    <nav class="w-16 flex flex-col items-center py-4 border-r border-sidebar-border bg-sidebar">
      <!-- Logo -->
      <div class="mb-6">
        <img 
          src="/ripple-logo.png" 
          alt="Ripple" 
          class="w-8 h-8 rounded-lg"
        />
      </div>

      <!-- Nav Items -->
      <div class="flex-1 flex flex-col gap-2">
        <button
          v-for="item in navItems"
          :key="item.id"
          :class="[
            'w-10 h-10 rounded-lg flex items-center justify-center transition-colors relative',
            activeNavItem === item.id
              ? 'bg-sidebar-accent text-sidebar-accent-foreground'
              : 'hover:bg-sidebar-accent/50'
          ]"
          :title="item.label"
          @click="handleNavClick(item.id)"
        >
          <span class="text-lg">{{ item.icon === 'robot' ? '🤖' : item.icon === 'hammer' ? '🔨' : item.icon === 'users' ? '👥' : item.icon === 'server' ? '🖥️' : item.icon === 'kanban' ? '📊' : '⚙️' }}</span>
          
          <!-- Badge -->
          <span
            v-if="item.badge?.count"
            class="absolute -top-1 -right-1 w-5 h-5 rounded-full bg-blue-500 text-white text-xs flex items-center justify-center"
          >
            {{ item.badge.count }}
          </span>
          <span
            v-else-if="item.badge?.status"
            :class="[
              'absolute -top-0.5 -right-0.5 w-2.5 h-2.5 rounded-full',
              item.badge.status === 'success' ? 'bg-green-500' : 'bg-blue-500'
            ]"
          />
        </button>
      </div>

      <!-- Expand/Collapse -->
      <button
        class="w-10 h-10 rounded-lg flex items-center justify-center hover:bg-sidebar-accent/50"
        @click="toggleExpanded"
      >
        <span>{{ isExpanded ? '❮' : '❯' }}</span>
      </button>
    </nav>

    <!-- Detail Sidebar -->
    <div
      v-if="isExpanded"
      class="w-72 border-r border-sidebar-border bg-sidebar overflow-y-auto"
    >
      <!-- Header -->
      <div class="p-4 border-b border-sidebar-border">
        <h2 class="font-semibold text-lg">{{ activeContent.title }}</h2>
      </div>

      <!-- Sections -->
      <div class="p-2">
        <div
          v-for="section in activeContent.sections"
          :key="section.title"
          class="mb-4"
        >
          <h3 class="px-2 py-1 text-xs font-medium text-muted-foreground uppercase tracking-wider">
            {{ section.title }}
          </h3>
          
          <div class="mt-1 space-y-0.5">
            <button
              v-for="item in section.items"
              :key="item.id"
              class="w-full flex items-center gap-3 px-2 py-2 rounded-md hover:bg-sidebar-accent/50 transition-colors text-left"
            >
              <span class="text-base">
                {{ item.icon === 'message-square' ? '💬' : item.icon === 'sparkles' ? '✨' : item.icon === 'robot' ? '🤖' : item.icon === 'check-circle' ? '✅' : item.icon === 'loader' ? '⏳' : item.icon === 'server' ? '🖥️' : item.icon === 'cpu' ? '💻' : item.icon === 'folder' ? '📁' : item.icon === 'check-square' ? '☑️' : item.icon === 'palette' ? '🎨' : item.icon === 'bell' ? '🔔' : '📌' }}
              </span>
              
              <div class="flex-1 min-w-0">
                <div class="flex items-center gap-2">
                  <span class="truncate font-medium">{{ item.label }}</span>
                  <span
                    v-if="item.status"
                    :class="['w-2 h-2 rounded-full', getStatusColor(item.status)]"
                  />
                </div>
                <span v-if="item.metadata" class="text-xs text-muted-foreground truncate block">
                  {{ item.metadata }}
                </span>
              </div>

              <!-- Badge -->
              <span
                v-if="item.badge?.count"
                class="px-1.5 py-0.5 text-xs rounded-full bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-200"
              >
                {{ item.badge.count }}
              </span>
            </button>
          </div>
        </div>
      </div>

      <!-- Kanban Connection Status -->
      <div v-if="activeNavItem === 'kanban'" class="p-4 border-t border-sidebar-border">
        <div class="flex items-center gap-2 text-sm">
          <span :class="['w-2 h-2 rounded-full', kanbanConnected ? 'bg-green-500' : 'bg-red-500']" />
          <span>{{ kanbanConnected ? 'Connected to Vibe Kanban' : 'Vibe Kanban not available' }}</span>
        </div>
      </div>
    </div>
  </aside>
</template>
