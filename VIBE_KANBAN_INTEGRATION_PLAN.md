# Vibe Kanban Integration Plan for Minimalist Sidebar

## Executive Summary

This document outlines the comprehensive plan to integrate vibe-kanban's task management and kanban board functionality into the minimalist_sidebar_react framework. The goal is to provide a unified, cross-platform task orchestration interface for AI coding agents within the Ripple ecosystem.

---

## 1. Current State Analysis

### 1.1 Vibe Kanban Overview

**Location**: `/home/ubuntu/ripple-env/frontend/vibe-kanban`

**Project Structure**:
```
vibe-kanban/
├── crates/                    # Rust backend
│   ├── server/               # API server (Axum-based)
│   ├── db/                   # SQLite database layer
│   ├── executors/            # Agent execution handlers
│   ├── services/             # Business logic services
│   ├── utils/                # Shared utilities
│   ├── local-deployment/     # Local deployment tooling
│   ├── deployment/           # Remote deployment
│   ├── remote/               # Remote server features
│   └── review/               # Code review features
├── frontend/                  # React/TypeScript frontend
│   ├── src/
│   │   ├── components/       # UI components
│   │   ├── contexts/         # React contexts
│   │   ├── hooks/            # Custom hooks
│   │   ├── pages/            # Route pages
│   │   ├── stores/           # Zustand stores
│   │   ├── styles/           # CSS/design tokens
│   │   └── utils/            # Frontend utilities
├── shared/                    # Shared TypeScript types
└── npx-cli/                   # NPX CLI distribution
```

**Technology Stack**:
- **Backend**: Rust (Axum, SQLx, SQLite)
- **Frontend**: React 18, TypeScript, Vite
- **State Management**: Zustand, TanStack Query
- **UI Framework**: Tailwind CSS, Radix UI
- **Other**: i18next, Framer Motion, CodeMirror

### 1.2 Minimalist Sidebar Overview

**Location**: `/home/ubuntu/ripple-env/frontend/minimalist_sidebar_react`

**Project Structure**:
```
minimalist_sidebar_react/
├── app/                       # Expo Router pages
│   ├── (drawer)/             # Drawer navigation screens
│   ├── (tabs)/               # Tab navigation screens
│   └── _layout.tsx           # Root layout
├── components/                # React Native components
│   └── sidebar/              # Sidebar components
├── contexts/                  # Context providers
├── data/                      # Static data/configs
├── hooks/                     # Custom hooks
├── services/                  # Business services
└── shared/                    # Shared utilities
```

**Technology Stack**:
- **Framework**: Expo/React Native (with Web support)
- **Backend**: tRPC server with Drizzle ORM
- **State**: TanStack Query, React Context
- **UI**: NativeWind (Tailwind for RN)
- **Navigation**: Expo Router, React Navigation

### 1.3 Key Capabilities to Integrate

From vibe-kanban:
1. **Kanban Board** - Drag-and-drop task management
2. **Task Status System** - todo, inprogress, inreview, done, cancelled
3. **Workspace Management** - Container-based isolated workspaces
4. **Session Management** - Agent execution sessions
5. **Execution Process Tracking** - Real-time agent monitoring
6. **Code Diff Viewing** - Git diff visualization
7. **Preview Browser** - Live preview of changes
8. **Git Operations** - Branch management, merging
9. **Multi-agent Support** - Multiple coding agents
10. **Approval System** - Human-in-the-loop approvals

---

## 2. Architecture Design

### 2.1 Integration Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     Minimalist Sidebar App                       │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Sidebar Nav   │  │   Main Content  │  │   Detail Panel  │ │
│  │                 │  │                 │  │                 │ │
│  │  - Workspaces   │  │  - Kanban Board │  │  - Task Details │ │
│  │  - AI Command   │  │  - Task List    │  │  - Logs Viewer  │ │
│  │  - Tasks        │  │  - Timeline     │  │  - Diff View    │ │
│  │  - Projects     │  │                 │  │  - Preview      │ │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘ │
│           │                    │                    │           │
│  ┌────────▼────────────────────▼────────────────────▼────────┐ │
│  │              Shared State Layer (Zustand/Context)          │ │
│  └─────────────────────────────┬──────────────────────────────┘ │
└────────────────────────────────┼────────────────────────────────┘
                                 │
┌────────────────────────────────▼────────────────────────────────┐
│                    API Layer (tRPC + REST)                       │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │  tRPC Client    │  │  REST Client    │  │  WebSocket      │ │
│  │  (internal)     │  │  (vibe-kanban)  │  │  (real-time)    │ │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘ │
└───────────┼────────────────────┼────────────────────┼───────────┘
            │                    │                    │
┌───────────▼────────────────────▼────────────────────▼───────────┐
│                      Backend Services                            │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                  Vibe Kanban Server (Rust)                  ││
│  │  - Task Management API                                       ││
│  │  - Workspace Management                                      ││
│  │  - Execution Process Control                                 ││
│  │  - Git Operations                                            ││
│  └─────────────────────────────────────────────────────────────┘│
│  ┌─────────────────────────────────────────────────────────────┐│
│  │            Minimalist Sidebar Server (Node.js/tRPC)         ││
│  │  - User Preferences                                          ││
│  │  - Session Management                                        ││
│  │  - Proxy to Vibe Kanban                                      ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Component Mapping

| Vibe Kanban Component | Minimalist Sidebar Equivalent | Integration Approach |
|-----------------------|-------------------------------|----------------------|
| `TaskKanbanBoard` | New `KanbanBoard` in sidebar | Adapt with RN drag-drop |
| `TaskCard` | New `TaskCard` component | Style with NativeWind |
| `TasksLayout` | Drawer + split view | Expo Router nested layouts |
| `WorkspacesLayout` | Sidebar panel content | WebView or native port |
| `TaskFollowUpSection` | Chat-style interaction | Native implementation |
| `DiffCard` | `DiffViewer` component | WebView for complex diffs |
| `PreviewBrowser` | `PreviewPanel` | WebView iframe |
| `NormalizedConversation` | Message list | Virtuoso → FlashList |

### 2.3 Data Flow

```
┌──────────────────────────────────────────────────────────────┐
│                      User Actions                             │
│  - Create Task          - Move Task          - View Logs      │
│  - Start Agent          - Approve Action     - View Diffs     │
└──────────────────────────────────┬───────────────────────────┘
                                   │
                                   ▼
┌──────────────────────────────────────────────────────────────┐
│                   React Query / tRPC Hooks                    │
│  - useTasks()           - useWorkspace()     - useProcess()   │
│  - useCreateTask()      - useApproval()      - useDiffs()     │
└──────────────────────────────────┬───────────────────────────┘
                                   │
                                   ▼
┌──────────────────────────────────────────────────────────────┐
│                      API Clients                              │
│  ┌─────────────────┐  ┌──────────────────┐  ┌──────────────┐ │
│  │ Vibe Kanban API │  │ tRPC Internal API│  │  WebSocket   │ │
│  │ /api/tasks/*    │  │ /trpc/*          │  │  /events/*   │ │
│  └────────┬────────┘  └────────┬─────────┘  └──────┬───────┘ │
└───────────┼────────────────────┼────────────────────┼─────────┘
            │                    │                    │
            ▼                    ▼                    ▼
┌──────────────────────────────────────────────────────────────┐
│                   Vibe Kanban Backend                         │
│  - SQLite Database      - Git Operations    - Agent Executor  │
└──────────────────────────────────────────────────────────────┘
```

---

## 3. Implementation Plan

### Phase 1: Foundation Setup (Week 1)

#### 3.1.1 Backend Integration Layer

**Task**: Create API client for vibe-kanban in minimalist_sidebar

```typescript
// lib/vibe-kanban/client.ts
import axios from 'axios';
import type { Task, Workspace, ExecutionProcess } from './types';

const VIBE_KANBAN_URL = process.env.VIBE_KANBAN_URL || 'http://localhost:3000';

export const vibeKanbanClient = axios.create({
  baseURL: `${VIBE_KANBAN_URL}/api`,
  headers: {
    'Content-Type': 'application/json',
  },
});

export const taskApi = {
  list: (projectId: string) => vibeKanbanClient.get<Task[]>(`/projects/${projectId}/tasks`),
  create: (data: CreateTask) => vibeKanbanClient.post<Task>('/tasks', data),
  update: (id: string, data: UpdateTask) => vibeKanbanClient.patch<Task>(`/tasks/${id}`, data),
  delete: (id: string) => vibeKanbanClient.delete(`/tasks/${id}`),
};

export const workspaceApi = {
  list: (taskId: string) => vibeKanbanClient.get<Workspace[]>(`/tasks/${taskId}/attempts`),
  create: (taskId: string, data: CreateWorkspace) => 
    vibeKanbanClient.post<Workspace>(`/tasks/${taskId}/attempts`, data),
  start: (id: string) => vibeKanbanClient.post(`/attempts/${id}/start`),
  stop: (id: string) => vibeKanbanClient.post(`/attempts/${id}/stop`),
};
```

**Task**: Add vibe-kanban routes to tRPC router

```typescript
// server/_core/routers/vibe-kanban.ts
import { z } from 'zod';
import { router, publicProcedure } from '../trpc';
import { taskApi, workspaceApi } from '@/lib/vibe-kanban/client';

export const vibeKanbanRouter = router({
  tasks: router({
    list: publicProcedure
      .input(z.object({ projectId: z.string() }))
      .query(async ({ input }) => {
        const { data } = await taskApi.list(input.projectId);
        return data;
      }),
    create: publicProcedure
      .input(z.object({ 
        projectId: z.string(),
        title: z.string(),
        description: z.string().optional(),
      }))
      .mutation(async ({ input }) => {
        const { data } = await taskApi.create(input);
        return data;
      }),
    updateStatus: publicProcedure
      .input(z.object({ 
        taskId: z.string(),
        status: z.enum(['todo', 'inprogress', 'inreview', 'done', 'cancelled']),
      }))
      .mutation(async ({ input }) => {
        const { data } = await taskApi.update(input.taskId, { status: input.status });
        return data;
      }),
  }),
  workspaces: router({
    // ... workspace procedures
  }),
});
```

#### 3.1.2 Type Sharing

**Task**: Copy and adapt shared types

```bash
# Create shared types directory
mkdir -p /home/ubuntu/ripple-env/frontend/minimalist_sidebar_react/shared/vibe-kanban

# Copy and adapt types
cp /home/ubuntu/ripple-env/frontend/vibe-kanban/shared/types.ts \
   /home/ubuntu/ripple-env/frontend/minimalist_sidebar_react/shared/vibe-kanban/types.ts
```

**Modifications needed**:
- Replace `Date` with `string` (JSON serialization)
- Remove `bigint` or convert to `number`
- Add React Native compatible exports

#### 3.1.3 Update Dependencies

Add to `package.json`:
```json
{
  "dependencies": {
    "axios": "^1.7.0",
    "@dnd-kit/core": "^6.3.1",
    "@dnd-kit/sortable": "^8.0.0",
    "react-native-flash-list": "^1.7.0",
    "react-native-webview": "^13.0.0"
  }
}
```

### Phase 2: Core Components (Week 2)

#### 3.2.1 Kanban Board Component

**File**: `components/kanban/KanbanBoard.tsx`

```tsx
import React from 'react';
import { View, ScrollView } from 'react-native';
import { DndContext, closestCenter } from '@dnd-kit/core';
import { SortableContext, verticalListSortingStrategy } from '@dnd-kit/sortable';
import { KanbanColumn } from './KanbanColumn';
import { TaskCard } from './TaskCard';
import type { Task, TaskStatus } from '@/shared/vibe-kanban/types';

interface KanbanBoardProps {
  tasks: Task[];
  onTaskMove: (taskId: string, newStatus: TaskStatus) => void;
  onTaskSelect: (task: Task) => void;
}

const COLUMNS: TaskStatus[] = ['todo', 'inprogress', 'inreview', 'done'];

export function KanbanBoard({ tasks, onTaskMove, onTaskSelect }: KanbanBoardProps) {
  const tasksByStatus = COLUMNS.reduce((acc, status) => {
    acc[status] = tasks.filter(t => t.status === status);
    return acc;
  }, {} as Record<TaskStatus, Task[]>);

  const handleDragEnd = (event: DragEndEvent) => {
    const { active, over } = event;
    if (over && active.id !== over.id) {
      const newStatus = over.data.current?.column as TaskStatus;
      onTaskMove(active.id as string, newStatus);
    }
  };

  return (
    <ScrollView horizontal showsHorizontalScrollIndicator={false}>
      <DndContext onDragEnd={handleDragEnd} collisionDetection={closestCenter}>
        <View className="flex-row gap-4 p-4">
          {COLUMNS.map(status => (
            <KanbanColumn key={status} status={status} count={tasksByStatus[status].length}>
              <SortableContext 
                items={tasksByStatus[status].map(t => t.id)}
                strategy={verticalListSortingStrategy}
              >
                {tasksByStatus[status].map(task => (
                  <TaskCard 
                    key={task.id} 
                    task={task} 
                    onPress={() => onTaskSelect(task)}
                  />
                ))}
              </SortableContext>
            </KanbanColumn>
          ))}
        </View>
      </DndContext>
    </ScrollView>
  );
}
```

#### 3.2.2 Task Card Component

**File**: `components/kanban/TaskCard.tsx`

```tsx
import React from 'react';
import { View, Text, Pressable } from 'react-native';
import { useSortable } from '@dnd-kit/sortable';
import { CSS } from '@dnd-kit/utilities';
import { cn } from '@/lib/utils';
import type { TaskWithAttemptStatus } from '@/shared/vibe-kanban/types';

interface TaskCardProps {
  task: TaskWithAttemptStatus;
  onPress: () => void;
}

export function TaskCard({ task, onPress }: TaskCardProps) {
  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
    isDragging,
  } = useSortable({ id: task.id });

  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
    opacity: isDragging ? 0.5 : 1,
  };

  return (
    <Pressable
      ref={setNodeRef}
      style={style}
      {...attributes}
      {...listeners}
      onPress={onPress}
      className={cn(
        "bg-surface p-3 rounded-lg border border-border mb-2",
        "active:scale-98 transition-transform",
        task.has_in_progress_attempt && "border-l-4 border-l-primary"
      )}
    >
      <Text className="text-foreground font-medium text-sm" numberOfLines={2}>
        {task.title}
      </Text>
      {task.description && (
        <Text className="text-muted text-xs mt-1" numberOfLines={1}>
          {task.description}
        </Text>
      )}
      <View className="flex-row items-center mt-2 gap-2">
        {task.has_in_progress_attempt && (
          <View className="bg-primary/20 px-2 py-0.5 rounded-full">
            <Text className="text-primary text-xs">Running</Text>
          </View>
        )}
        {task.last_attempt_failed && (
          <View className="bg-error/20 px-2 py-0.5 rounded-full">
            <Text className="text-error text-xs">Failed</Text>
          </View>
        )}
        <Text className="text-muted text-xs">{task.executor}</Text>
      </View>
    </Pressable>
  );
}
```

#### 3.2.3 Task Details Panel

**File**: `components/kanban/TaskDetailsPanel.tsx`

```tsx
import React from 'react';
import { View, Text, ScrollView, Pressable } from 'react-native';
import { WebView } from 'react-native-webview';
import { useTask, useTaskWorkspaces, useStartWorkspace } from '@/hooks/vibe-kanban';
import { AgentSelector } from './AgentSelector';
import { WorkspaceList } from './WorkspaceList';
import { LogViewer } from '../logs/LogViewer';

interface TaskDetailsPanelProps {
  taskId: string;
  onClose: () => void;
}

export function TaskDetailsPanel({ taskId, onClose }: TaskDetailsPanelProps) {
  const { data: task, isLoading } = useTask(taskId);
  const { data: workspaces } = useTaskWorkspaces(taskId);
  const startWorkspace = useStartWorkspace();
  
  const [activeTab, setActiveTab] = React.useState<'details' | 'logs' | 'diffs'>('details');

  if (isLoading || !task) {
    return <LoadingSkeleton />;
  }

  return (
    <View className="flex-1 bg-background">
      {/* Header */}
      <View className="p-4 border-b border-border">
        <View className="flex-row items-center justify-between">
          <Text className="text-foreground text-lg font-semibold" numberOfLines={1}>
            {task.title}
          </Text>
          <Pressable onPress={onClose} className="p-2">
            <XIcon className="text-muted" />
          </Pressable>
        </View>
        
        {/* Tab Bar */}
        <View className="flex-row mt-4 gap-2">
          {['details', 'logs', 'diffs'].map(tab => (
            <Pressable
              key={tab}
              onPress={() => setActiveTab(tab as any)}
              className={cn(
                "px-4 py-2 rounded-lg",
                activeTab === tab ? "bg-primary" : "bg-surface"
              )}
            >
              <Text className={cn(
                "text-sm capitalize",
                activeTab === tab ? "text-white" : "text-muted"
              )}>
                {tab}
              </Text>
            </Pressable>
          ))}
        </View>
      </View>

      {/* Content */}
      <ScrollView className="flex-1 p-4">
        {activeTab === 'details' && (
          <View className="gap-4">
            <View>
              <Text className="text-muted text-xs uppercase mb-1">Description</Text>
              <Text className="text-foreground">{task.description || 'No description'}</Text>
            </View>
            
            <AgentSelector taskId={taskId} currentAgent={task.executor} />
            
            <View>
              <Text className="text-muted text-xs uppercase mb-2">Workspaces</Text>
              <WorkspaceList workspaces={workspaces || []} taskId={taskId} />
            </View>
          </View>
        )}
        
        {activeTab === 'logs' && (
          <LogViewer taskId={taskId} />
        )}
        
        {activeTab === 'diffs' && (
          <DiffViewer taskId={taskId} />
        )}
      </ScrollView>

      {/* Action Bar */}
      <View className="p-4 border-t border-border">
        <Pressable
          onPress={() => startWorkspace.mutate({ taskId })}
          className="bg-primary py-3 rounded-lg items-center"
          disabled={startWorkspace.isPending}
        >
          <Text className="text-white font-medium">
            {startWorkspace.isPending ? 'Starting...' : 'Start Agent'}
          </Text>
        </Pressable>
      </View>
    </View>
  );
}
```

### Phase 3: Sidebar Integration (Week 3)

#### 3.3.1 Update Sidebar Panel Data

**File**: `data/sidebar-panels.ts`

Add vibe-kanban panels:

```typescript
export const sidebarPanels: Record<string, SidebarPanel> = {
  // ... existing panels ...
  
  'tasks': {
    id: 'tasks',
    title: 'Tasks',
    icon: 'CheckSquare',
    sections: [
      {
        title: 'Kanban Board',
        items: [
          { id: 'all-tasks', label: 'All Tasks', icon: 'Layout', route: '/(drawer)/tasks' },
          { id: 'my-tasks', label: 'My Tasks', icon: 'User', badge: { count: 5, color: 'red' } },
          { id: 'in-progress', label: 'In Progress', icon: 'Loader', badge: { count: 2, color: 'blue' } },
        ]
      },
      {
        title: 'Quick Actions',
        items: [
          { id: 'new-task', label: 'Create Task', icon: 'Plus', action: 'create-task' },
          { id: 'import', label: 'Import from GitHub', icon: 'Github', action: 'import-github' },
        ]
      }
    ]
  },
  
  'workspaces': {
    id: 'workspaces',
    title: 'Workspaces',
    icon: 'Layers',
    sections: [
      {
        title: 'Active Workspaces',
        items: [
          { id: 'running', label: 'Running', icon: 'Play', badge: { count: 1, color: 'green' } },
          { id: 'paused', label: 'Paused', icon: 'Pause' },
        ]
      },
      {
        title: 'Recent',
        items: [
          // Dynamic items from API
        ]
      }
    ]
  },
  
  'agents': {
    id: 'agents',
    title: 'AI Agents',
    icon: 'Bot',
    sections: [
      {
        title: 'Coding Agents',
        items: [
          { id: 'claude', label: 'Claude Code', icon: 'Sparkles', status: 'online' },
          { id: 'codex', label: 'Codex', icon: 'Code', status: 'offline' },
          { id: 'gemini', label: 'Gemini CLI', icon: 'Gem', status: 'online' },
        ]
      },
      {
        title: 'Configuration',
        items: [
          { id: 'mcp-servers', label: 'MCP Servers', icon: 'Server', route: '/settings/mcp' },
          { id: 'agent-settings', label: 'Agent Settings', icon: 'Settings', route: '/settings/agents' },
        ]
      }
    ]
  }
};
```

#### 3.3.2 Create Tasks Screen

**File**: `app/(drawer)/tasks.tsx`

```tsx
import React, { useState } from 'react';
import { View } from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { useProjects, useTasks } from '@/hooks/vibe-kanban';
import { KanbanBoard } from '@/components/kanban/KanbanBoard';
import { TaskDetailsPanel } from '@/components/kanban/TaskDetailsPanel';
import { ProjectSelector } from '@/components/kanban/ProjectSelector';
import { CreateTaskModal } from '@/components/kanban/CreateTaskModal';
import { DrawerToggleButton } from '@/components/sidebar/DrawerToggleButton';

export default function TasksScreen() {
  const { data: projects } = useProjects();
  const [selectedProjectId, setSelectedProjectId] = useState<string | null>(null);
  const [selectedTaskId, setSelectedTaskId] = useState<string | null>(null);
  const [showCreateModal, setShowCreateModal] = useState(false);
  
  const { data: tasks, refetch } = useTasks(selectedProjectId);
  const updateTaskStatus = useUpdateTaskStatus();

  const handleTaskMove = async (taskId: string, newStatus: TaskStatus) => {
    await updateTaskStatus.mutateAsync({ taskId, status: newStatus });
  };

  return (
    <SafeAreaView className="flex-1 bg-background">
      {/* Header */}
      <View className="flex-row items-center p-4 border-b border-border">
        <DrawerToggleButton />
        <ProjectSelector
          projects={projects || []}
          selectedId={selectedProjectId}
          onSelect={setSelectedProjectId}
        />
        <Pressable 
          onPress={() => setShowCreateModal(true)}
          className="ml-auto bg-primary px-4 py-2 rounded-lg"
        >
          <Text className="text-white">New Task</Text>
        </Pressable>
      </View>

      {/* Content */}
      <View className="flex-1 flex-row">
        <View className={selectedTaskId ? 'flex-[2]' : 'flex-1'}>
          <KanbanBoard
            tasks={tasks || []}
            onTaskMove={handleTaskMove}
            onTaskSelect={(task) => setSelectedTaskId(task.id)}
          />
        </View>
        
        {selectedTaskId && (
          <View className="flex-1 border-l border-border">
            <TaskDetailsPanel
              taskId={selectedTaskId}
              onClose={() => setSelectedTaskId(null)}
            />
          </View>
        )}
      </View>

      <CreateTaskModal
        visible={showCreateModal}
        projectId={selectedProjectId}
        onClose={() => setShowCreateModal(false)}
        onCreated={() => {
          setShowCreateModal(false);
          refetch();
        }}
      />
    </SafeAreaView>
  );
}
```

### Phase 4: Real-time Features (Week 4)

#### 3.4.1 WebSocket Integration

**File**: `lib/vibe-kanban/websocket.ts`

```typescript
import { useEffect, useCallback } from 'react';
import { useQueryClient } from '@tanstack/react-query';

const VIBE_KANBAN_WS_URL = process.env.VIBE_KANBAN_WS_URL || 'ws://localhost:3000/events';

export function useVibeKanbanEvents(projectId: string | null) {
  const queryClient = useQueryClient();

  useEffect(() => {
    if (!projectId) return;

    const ws = new WebSocket(`${VIBE_KANBAN_WS_URL}?project_id=${projectId}`);

    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      
      switch (data.type) {
        case 'task_updated':
          queryClient.invalidateQueries({ queryKey: ['tasks', projectId] });
          break;
        case 'execution_process_updated':
          queryClient.invalidateQueries({ queryKey: ['execution-process', data.processId] });
          break;
        case 'workspace_updated':
          queryClient.invalidateQueries({ queryKey: ['workspaces'] });
          break;
        case 'approval_requested':
          // Show approval notification
          showApprovalNotification(data.approval);
          break;
      }
    };

    return () => ws.close();
  }, [projectId, queryClient]);
}
```

#### 3.4.2 Log Streaming

**File**: `components/logs/LogViewer.tsx`

```tsx
import React, { useEffect, useRef, useState } from 'react';
import { View, Text, ScrollView, Platform } from 'react-native';
import { WebView } from 'react-native-webview';
import { useLogStream } from '@/hooks/vibe-kanban';
import { AnsiParser } from '@/lib/ansi-parser';

interface LogViewerProps {
  taskId: string;
  workspaceId?: string;
}

export function LogViewer({ taskId, workspaceId }: LogViewerProps) {
  const scrollRef = useRef<ScrollView>(null);
  const { logs, isStreaming } = useLogStream(taskId, workspaceId);
  const [autoScroll, setAutoScroll] = useState(true);

  useEffect(() => {
    if (autoScroll && scrollRef.current) {
      scrollRef.current.scrollToEnd({ animated: true });
    }
  }, [logs, autoScroll]);

  // For complex ANSI rendering, use WebView on mobile
  if (Platform.OS !== 'web') {
    return (
      <WebView
        source={{
          html: `
            <html>
              <head>
                <style>
                  body { 
                    background: #0a0a0a; 
                    color: #e0e0e0;
                    font-family: monospace;
                    font-size: 12px;
                    padding: 8px;
                    margin: 0;
                  }
                  .line { white-space: pre-wrap; }
                </style>
              </head>
              <body>
                ${logs.map(log => `<div class="line">${AnsiParser.toHtml(log)}</div>`).join('')}
              </body>
            </html>
          `
        }}
        style={{ flex: 1, backgroundColor: '#0a0a0a' }}
      />
    );
  }

  return (
    <ScrollView
      ref={scrollRef}
      className="flex-1 bg-black p-2"
      onScroll={(e) => {
        const { layoutMeasurement, contentOffset, contentSize } = e.nativeEvent;
        const isAtBottom = layoutMeasurement.height + contentOffset.y >= contentSize.height - 20;
        setAutoScroll(isAtBottom);
      }}
    >
      {logs.map((log, index) => (
        <Text key={index} className="text-white font-mono text-xs">
          {log}
        </Text>
      ))}
      {isStreaming && (
        <View className="flex-row items-center gap-2 mt-2">
          <View className="w-2 h-2 bg-primary rounded-full animate-pulse" />
          <Text className="text-muted text-xs">Streaming logs...</Text>
        </View>
      )}
    </ScrollView>
  );
}
```

### Phase 5: Advanced Features (Week 5)

#### 3.5.1 Diff Viewer

Use WebView with vibe-kanban's existing diff viewer:

```tsx
// components/diff/DiffViewer.tsx
import { WebView } from 'react-native-webview';

interface DiffViewerProps {
  workspaceId: string;
  repoId: string;
}

export function DiffViewer({ workspaceId, repoId }: DiffViewerProps) {
  const VIBE_KANBAN_URL = process.env.VIBE_KANBAN_URL;
  
  return (
    <WebView
      source={{ uri: `${VIBE_KANBAN_URL}/workspaces/${workspaceId}/diffs?repo=${repoId}&embed=true` }}
      style={{ flex: 1 }}
    />
  );
}
```

#### 3.5.2 Preview Browser

```tsx
// components/preview/PreviewBrowser.tsx
import { WebView } from 'react-native-webview';
import { useState } from 'react';

interface PreviewBrowserProps {
  url: string;
  onUrlChange?: (url: string) => void;
}

export function PreviewBrowser({ url, onUrlChange }: PreviewBrowserProps) {
  const [currentUrl, setCurrentUrl] = useState(url);

  return (
    <View className="flex-1">
      <View className="flex-row items-center p-2 bg-surface border-b border-border">
        <TextInput
          value={currentUrl}
          onChangeText={setCurrentUrl}
          onSubmitEditing={() => onUrlChange?.(currentUrl)}
          className="flex-1 bg-background px-3 py-2 rounded text-foreground"
        />
        <Pressable onPress={() => {/* refresh */}} className="p-2 ml-2">
          <RefreshIcon className="text-muted" />
        </Pressable>
      </View>
      <WebView source={{ uri: currentUrl }} style={{ flex: 1 }} />
    </View>
  );
}
```

---

## 4. Required Changes to Minimalist Sidebar

### 4.1 Package.json Updates

```json
{
  "dependencies": {
    // Add these
    "axios": "^1.7.0",
    "@dnd-kit/core": "^6.3.1",
    "@dnd-kit/sortable": "^8.0.0",
    "@dnd-kit/utilities": "^3.2.2",
    "react-native-webview": "^13.0.0",
    "fancy-ansi": "^0.1.3"
  }
}
```

### 4.2 App Configuration Updates

**File**: `app.config.ts`

Add WebView plugin and environment variables:

```typescript
export default {
  // ...
  plugins: [
    // ...
    'react-native-webview',
  ],
  extra: {
    vibeKanbanUrl: process.env.VIBE_KANBAN_URL || 'http://localhost:3000',
    vibeKanbanWsUrl: process.env.VIBE_KANBAN_WS_URL || 'ws://localhost:3000/events',
  },
};
```

### 4.3 Environment Setup

**File**: `.env.example`

```bash
# Vibe Kanban Integration
VIBE_KANBAN_URL=http://localhost:3000
VIBE_KANBAN_WS_URL=ws://localhost:3000/events

# Backend Server
SERVER_PORT=8080
```

### 4.4 Directory Structure Changes

```
minimalist_sidebar_react/
├── components/
│   ├── kanban/                 # NEW: Kanban components
│   │   ├── KanbanBoard.tsx
│   │   ├── KanbanColumn.tsx
│   │   ├── TaskCard.tsx
│   │   ├── TaskDetailsPanel.tsx
│   │   ├── AgentSelector.tsx
│   │   ├── WorkspaceList.tsx
│   │   └── CreateTaskModal.tsx
│   ├── logs/                   # NEW: Log viewing components
│   │   ├── LogViewer.tsx
│   │   └── AnsiRenderer.tsx
│   ├── diff/                   # NEW: Diff viewing components
│   │   └── DiffViewer.tsx
│   ├── preview/                # NEW: Preview components
│   │   └── PreviewBrowser.tsx
│   └── sidebar/                # EXISTING: Update panel data
├── hooks/
│   └── vibe-kanban/            # NEW: API hooks
│       ├── useTasks.ts
│       ├── useWorkspaces.ts
│       ├── useExecutionProcess.ts
│       └── index.ts
├── lib/
│   └── vibe-kanban/            # NEW: API client
│       ├── client.ts
│       ├── websocket.ts
│       └── types.ts
├── shared/
│   └── vibe-kanban/            # NEW: Shared types
│       └── types.ts
└── app/
    └── (drawer)/
        ├── tasks.tsx           # NEW: Tasks screen
        ├── workspaces.tsx      # NEW: Workspaces screen
        └── agents.tsx          # NEW: Agents screen
```

---

## 5. Dependencies & Prerequisites

### 5.1 Runtime Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| Vibe Kanban Server | 0.0.152+ | Backend API |
| Node.js | ≥18 | tRPC server |
| SQLite | Any | Database |
| Git | ≥2.0 | Repository operations |

### 5.2 Development Dependencies

| Dependency | Purpose |
|------------|---------|
| Expo SDK 54 | React Native development |
| TypeScript 5.x | Type safety |
| pnpm | Package management |

### 5.3 External Services (Optional)

- **GitHub OAuth** - For GitHub integration
- **PostHog** - Analytics (optional)
- **Sentry** - Error tracking (optional)

### 5.4 Configuration Requirements

1. **Vibe Kanban Server Running**: Must be accessible at configured URL
2. **Network Access**: Mobile app must be able to reach server
3. **Git Configuration**: For repository operations
4. **Agent Credentials**: API keys for coding agents (Claude, Codex, etc.)

---

## 6. Testing Strategy

### 6.1 Unit Tests

```typescript
// __tests__/kanban/TaskCard.test.tsx
import { render, fireEvent } from '@testing-library/react-native';
import { TaskCard } from '@/components/kanban/TaskCard';

describe('TaskCard', () => {
  it('renders task title', () => {
    const task = { id: '1', title: 'Test Task', status: 'todo' };
    const { getByText } = render(<TaskCard task={task} onPress={() => {}} />);
    expect(getByText('Test Task')).toBeTruthy();
  });

  it('shows running badge when in progress', () => {
    const task = { id: '1', title: 'Test', has_in_progress_attempt: true };
    const { getByText } = render(<TaskCard task={task} onPress={() => {}} />);
    expect(getByText('Running')).toBeTruthy();
  });
});
```

### 6.2 Integration Tests

```typescript
// __tests__/integration/kanban-flow.test.tsx
import { renderWithProviders } from '@/test-utils';
import TasksScreen from '@/app/(drawer)/tasks';

describe('Kanban Flow', () => {
  it('creates and moves task through columns', async () => {
    const { getByText, findByText } = renderWithProviders(<TasksScreen />);
    
    // Create task
    fireEvent.press(getByText('New Task'));
    // ... fill form
    
    // Verify in todo column
    expect(await findByText('Test Task')).toBeTruthy();
    
    // Drag to inprogress
    // ... drag simulation
    
    // Verify moved
  });
});
```

### 6.3 E2E Tests

```typescript
// e2e/kanban.test.ts
describe('Kanban E2E', () => {
  beforeAll(async () => {
    await device.launchApp();
  });

  it('should complete full task workflow', async () => {
    // Navigate to tasks
    await element(by.id('sidebar-tasks')).tap();
    
    // Create task
    await element(by.id('create-task')).tap();
    await element(by.id('task-title')).typeText('E2E Test Task');
    await element(by.id('submit-task')).tap();
    
    // Start agent
    await element(by.text('E2E Test Task')).tap();
    await element(by.id('start-agent')).tap();
    
    // Wait for completion
    await waitFor(element(by.id('task-status-done')))
      .toBeVisible()
      .withTimeout(60000);
  });
});
```

### 6.4 API Tests

```typescript
// __tests__/api/vibe-kanban.test.ts
import { taskApi, workspaceApi } from '@/lib/vibe-kanban/client';

describe('Vibe Kanban API', () => {
  it('lists tasks for project', async () => {
    const { data } = await taskApi.list('test-project-id');
    expect(Array.isArray(data)).toBe(true);
  });

  it('creates task', async () => {
    const { data } = await taskApi.create({
      project_id: 'test-project',
      title: 'API Test Task',
    });
    expect(data.id).toBeDefined();
    expect(data.title).toBe('API Test Task');
  });
});
```

### 6.5 Manual Testing Checklist

- [ ] Task CRUD operations
- [ ] Drag-and-drop task movement
- [ ] Agent selection and start
- [ ] Log streaming in real-time
- [ ] Diff viewing
- [ ] Preview browser
- [ ] WebSocket reconnection
- [ ] Offline handling
- [ ] Cross-platform (iOS, Android, Web)

---

## 7. Rollout Plan

### Week 1: Foundation
- [ ] Set up API client
- [ ] Import types
- [ ] Create tRPC routes
- [ ] Basic connectivity test

### Week 2: Core Components
- [ ] KanbanBoard component
- [ ] TaskCard component
- [ ] TaskDetailsPanel
- [ ] Basic navigation

### Week 3: Sidebar Integration
- [ ] Update sidebar panels
- [ ] Create tasks screen
- [ ] Wire up navigation
- [ ] Test flow

### Week 4: Real-time
- [ ] WebSocket integration
- [ ] Log streaming
- [ ] Real-time updates
- [ ] Notifications

### Week 5: Advanced
- [ ] Diff viewer
- [ ] Preview browser
- [ ] Agent controls
- [ ] Polish UI

### Week 6: Testing & Polish
- [ ] Write tests
- [ ] Fix bugs
- [ ] Performance optimization
- [ ] Documentation

---

## 8. Risk Assessment

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| WebView performance on mobile | High | Medium | Use native components where possible |
| WebSocket instability | Medium | Medium | Implement reconnection logic |
| Type incompatibilities | Low | High | Careful type adaptation |
| CORS issues | Medium | Medium | Configure server properly |
| Offline functionality | Medium | Low | Implement caching layer |

---

## 9. Success Metrics

1. **Functionality**: All vibe-kanban features accessible from sidebar
2. **Performance**: <100ms response for UI interactions
3. **Reliability**: 99% uptime for WebSocket connections
4. **UX**: Complete task workflow without leaving app
5. **Test Coverage**: >80% for new components

---

## 10. Next Steps

1. **Immediate**: Review this plan with team
2. **This Week**: Start Phase 1 foundation work
3. **Ongoing**: Daily standups to track progress
4. **Milestone**: Demo integration at end of Week 3

---

*Document Version: 1.0*
*Last Updated: January 14, 2026*
*Author: DeepAgent*
