import { useState, useMemo, useEffect } from "react";
import { motion, AnimatePresence } from "framer-motion";
import {
  Search, Plus, Filter, SortAsc, LayoutGrid, List, Calendar as CalendarIcon,
  BarChart3, Star, Clock, Users, AlertCircle, CheckCircle2, XCircle,
  Loader2, Link as LinkIcon, ChevronRight, ChevronDown
} from "lucide-react";
import { trpc } from "@/lib/trpc";
import { toast } from "sonner";
import TaskDialogs from "@/components/tasks/TaskDialogs";
import {
  DndContext,
  DragEndEvent,
  DragOverlay,
  DragStartEvent,
  PointerSensor,
  useSensor,
  useSensors,
} from "@dnd-kit/core";

// Types matching the backend schema
export type TaskStatus = "todo" | "in_progress" | "review" | "done" | "cancelled";
export type TaskPriority = "low" | "medium" | "high" | "urgent";
export type ViewMode = "kanban" | "list" | "calendar" | "analytics";

export interface Task {
  id: number;
  userId: number;
  title: string;
  description?: string | null;
  status: TaskStatus;
  priority: TaskPriority;
  assignee?: string | null;
  dueDate?: Date | null;
  tags?: string[];
  estimatedTime?: number | null;
  progress: number;
  starred: boolean;
  parentTaskId?: number | null;
  sharedTaskId?: string | null;
  projectId?: string | null;
  hasInProgressAttempt: boolean;
  lastAttemptFailed: boolean;
  executor?: string | null;
  createdAt: Date;
  updatedAt: Date;
  // Relations
  comments?: TaskComment[];
  activities?: TaskActivity[];
  timeEntries?: TaskTimeEntry[];
  dependencies?: number[];
  subtasks?: Task[];
}

export interface TaskComment {
  id: number;
  taskId: number;
  author: string;
  content: string;
  avatar?: string | null;
  createdAt: Date;
}

export interface TaskActivity {
  id: number;
  taskId: number;
  type: "created" | "updated" | "commented" | "status_changed" | "assigned";
  user: string;
  description: string;
  createdAt: Date;
}

export interface TaskTimeEntry {
  id: number;
  taskId: number;
  duration: number;
  description: string;
  createdAt: Date;
}

const COLUMNS: { id: TaskStatus; title: string; color: string }[] = [
  { id: "todo", title: "To Do", color: "bg-gray-500" },
  { id: "in_progress", title: "In Progress", color: "bg-blue-500" },
  { id: "review", title: "Review", color: "bg-purple-500" },
  { id: "done", title: "Done", color: "bg-green-500" },
  { id: "cancelled", title: "Cancelled", color: "bg-red-500" },
];

const PRIORITY_COLORS = {
  low: "text-gray-400 border-gray-400",
  medium: "text-blue-400 border-blue-400",
  high: "text-orange-400 border-orange-400",
  urgent: "text-red-400 border-red-400",
};

export default function TasksEnhancedV3() {
  const [viewMode, setViewMode] = useState<ViewMode>("kanban");
  const [searchQuery, setSearchQuery] = useState("");
  const [filterPriority, setFilterPriority] = useState<TaskPriority | "all">("all");
  const [sortBy, setSortBy] = useState<"dueDate" | "priority" | "created" | "updated">("created");
  const [selectedTask, setSelectedTask] = useState<Task | null>(null);
  const [isCreateDialogOpen, setIsCreateDialogOpen] = useState(false);
  const [createDialogStatus, setCreateDialogStatus] = useState<TaskStatus | undefined>();
  const [activeId, setActiveId] = useState<number | null>(null);
  const [expandedTasks, setExpandedTasks] = useState<Set<number>>(new Set());

  // tRPC queries and mutations
  const { data: tasks = [], isLoading, refetch } = trpc.tasks.list.useQuery();
  const { data: analytics } = trpc.tasks.getAnalytics.useQuery();
  
  const createTaskMutation = trpc.tasks.create.useMutation({
    onSuccess: () => {
      toast.success("Task created successfully");
      refetch();
    },
    onError: (error) => {
      toast.error(`Failed to create task: ${error.message}`);
    },
  });

  const updateTaskMutation = trpc.tasks.update.useMutation({
    onSuccess: () => {
      toast.success("Task updated successfully");
      refetch();
    },
    onError: (error) => {
      toast.error(`Failed to update task: ${error.message}`);
    },
  });

  const deleteTaskMutation = trpc.tasks.delete.useMutation({
    onSuccess: () => {
      toast.success("Task deleted successfully");
      refetch();
    },
    onError: (error) => {
      toast.error(`Failed to delete task: ${error.message}`);
    },
  });

  const bulkUpdateStatusMutation = trpc.tasks.bulkUpdateStatus.useMutation({
    onSuccess: () => {
      refetch();
    },
    onError: (error) => {
      toast.error(`Failed to update task status: ${error.message}`);
    },
  });

  const addCommentMutation = trpc.tasks.addComment.useMutation({
    onSuccess: () => {
      toast.success("Comment added");
      refetch();
    },
  });

  const addTimeEntryMutation = trpc.tasks.addTimeEntry.useMutation({
    onSuccess: () => {
      toast.success("Time logged");
      refetch();
    },
  });

  // Drag and drop sensors
  const sensors = useSensors(
    useSensor(PointerSensor, {
      activationConstraint: {
        distance: 8,
      },
    })
  );

  // Filter and sort tasks
  const filteredTasks = useMemo(() => {
    let result = tasks;

    // Search filter
    if (searchQuery) {
      const query = searchQuery.toLowerCase();
      result = result.filter(
        (task) =>
          task.title.toLowerCase().includes(query) ||
          task.description?.toLowerCase().includes(query) ||
          task.tags?.some((tag) => tag.toLowerCase().includes(query))
      );
    }

    // Priority filter
    if (filterPriority !== "all") {
      result = result.filter((task) => task.priority === filterPriority);
    }

    // Sort
    result = [...result].sort((a, b) => {
      switch (sortBy) {
        case "dueDate":
          if (!a.dueDate) return 1;
          if (!b.dueDate) return -1;
          return new Date(a.dueDate).getTime() - new Date(b.dueDate).getTime();
        case "priority": {
          const priorityOrder = { urgent: 0, high: 1, medium: 2, low: 3 };
          return priorityOrder[a.priority] - priorityOrder[b.priority];
        }
        case "created":
          return new Date(b.createdAt).getTime() - new Date(a.createdAt).getTime();
        case "updated":
          return new Date(b.updatedAt).getTime() - new Date(a.updatedAt).getTime();
        default:
          return 0;
      }
    });

    return result;
  }, [tasks, searchQuery, filterPriority, sortBy]);

  // Group tasks by status for kanban view
  const tasksByStatus = useMemo(() => {
    const grouped: Record<TaskStatus, Task[]> = {
      todo: [],
      in_progress: [],
      review: [],
      done: [],
      cancelled: [],
    };

    filteredTasks.forEach((task) => {
      // Only show parent tasks (not subtasks) in kanban columns
      if (!task.parentTaskId) {
        grouped[task.status].push(task);
      }
    });

    return grouped;
  }, [filteredTasks]);

  // Handle drag start
  const handleDragStart = (event: DragStartEvent) => {
    setActiveId(event.active.id as number);
  };

  // Handle drag end
  const handleDragEnd = (event: DragEndEvent) => {
    const { active, over } = event;

    if (!over) {
      setActiveId(null);
      return;
    }

    const taskId = active.id as number;
    const newStatus = over.id as TaskStatus;

    const task = tasks.find((t) => t.id === taskId);
    if (task && task.status !== newStatus) {
      // Optimistically update UI
      updateTaskMutation.mutate({
        id: taskId,
        status: newStatus,
      });
    }

    setActiveId(null);
  };

  // Handle task creation
  const handleCreateTask = (data: Partial<Task>) => {
    createTaskMutation.mutate({
      title: data.title!,
      description: data.description,
      status: data.status || createDialogStatus,
      priority: data.priority,
      assignee: data.assignee,
      dueDate: data.dueDate ? new Date(data.dueDate) : undefined,
      tags: data.tags,
      estimatedTime: data.estimatedTime,
      progress: data.progress,
      starred: data.starred,
      parentTaskId: data.parentTaskId,
      projectId: data.projectId,
    });
    setIsCreateDialogOpen(false);
    setCreateDialogStatus(undefined);
  };

  // Handle task update
  const handleUpdateTask = (data: Partial<Task>) => {
    if (!selectedTask) return;

    updateTaskMutation.mutate({
      id: selectedTask.id,
      ...data,
      dueDate: data.dueDate ? new Date(data.dueDate) : undefined,
    });
  };

  // Handle task deletion
  const handleDeleteTask = (taskId: number) => {
    deleteTaskMutation.mutate({ id: taskId });
    setSelectedTask(null);
  };

  // Handle comment addition
  const handleAddComment = (taskId: number, content: string) => {
    addCommentMutation.mutate({
      taskId,
      author: "Current User", // This will be replaced by backend with actual user
      content,
    });
  };

  // Handle time entry addition
  const handleAddTimeEntry = (taskId: number, duration: number, description: string) => {
    addTimeEntryMutation.mutate({
      taskId,
      duration,
      description,
    });
  };

  // Toggle subtask expansion
  const toggleExpanded = (taskId: number) => {
    setExpandedTasks((prev) => {
      const next = new Set(prev);
      if (next.has(taskId)) {
        next.delete(taskId);
      } else {
        next.add(taskId);
      }
      return next;
    });
  };

  // Get active task for drag overlay
  const activeTask = activeId ? tasks.find((t) => t.id === activeId) : null;

  if (isLoading) {
    return (
      <div className="flex items-center justify-center h-screen bg-[#0A0A0A]">
        <Loader2 className="w-8 h-8 text-[#00D4FF] animate-spin" />
      </div>
    );
  }

  return (
    <div className="flex flex-col h-screen bg-[#0A0A0A] text-white">
      {/* Header */}
      <div className="flex items-center justify-between px-6 py-4 border-b border-white/10">
        <div>
          <h1 className="text-2xl font-bold">Tasks</h1>
          <p className="text-sm text-gray-400 mt-1">
            {analytics?.total || 0} tasks · {analytics?.completed || 0} completed
          </p>
        </div>

        <div className="flex items-center gap-3">
          {/* Search */}
          <div className="relative">
            <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-gray-400" />
            <input
              type="text"
              placeholder="Search tasks..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              className="pl-10 pr-4 py-2 bg-white/5 border border-white/10 rounded-lg focus:outline-none focus:border-[#00D4FF] w-64"
            />
          </div>

          {/* Filter */}
          <select
            value={filterPriority}
            onChange={(e) => setFilterPriority(e.target.value as TaskPriority | "all")}
            className="px-4 py-2 bg-white/5 border border-white/10 rounded-lg focus:outline-none focus:border-[#00D4FF]"
          >
            <option value="all">All Priorities</option>
            <option value="urgent">Urgent</option>
            <option value="high">High</option>
            <option value="medium">Medium</option>
            <option value="low">Low</option>
          </select>

          {/* Sort */}
          <select
            value={sortBy}
            onChange={(e) => setSortBy(e.target.value as typeof sortBy)}
            className="px-4 py-2 bg-white/5 border border-white/10 rounded-lg focus:outline-none focus:border-[#00D4FF]"
          >
            <option value="created">Sort by Created</option>
            <option value="updated">Sort by Updated</option>
            <option value="dueDate">Sort by Due Date</option>
            <option value="priority">Sort by Priority</option>
          </select>

          {/* View Mode */}
          <div className="flex gap-1 p-1 bg-white/5 rounded-lg">
            <button
              onClick={() => setViewMode("kanban")}
              className={`p-2 rounded ${viewMode === "kanban" ? "bg-[#00D4FF] text-black" : "text-gray-400 hover:text-white"}`}
            >
              <LayoutGrid className="w-4 h-4" />
            </button>
            <button
              onClick={() => setViewMode("list")}
              className={`p-2 rounded ${viewMode === "list" ? "bg-[#00D4FF] text-black" : "text-gray-400 hover:text-white"}`}
            >
              <List className="w-4 h-4" />
            </button>
            <button
              onClick={() => setViewMode("calendar")}
              className={`p-2 rounded ${viewMode === "calendar" ? "bg-[#00D4FF] text-black" : "text-gray-400 hover:text-white"}`}
            >
              <CalendarIcon className="w-4 h-4" />
            </button>
            <button
              onClick={() => setViewMode("analytics")}
              className={`p-2 rounded ${viewMode === "analytics" ? "bg-[#00D4FF] text-black" : "text-gray-400 hover:text-white"}`}
            >
              <BarChart3 className="w-4 h-4" />
            </button>
          </div>

          {/* Create Task */}
          <button
            onClick={() => {
              setCreateDialogStatus(undefined);
              setIsCreateDialogOpen(true);
            }}
            className="flex items-center gap-2 px-4 py-2 bg-[#00D4FF] text-black rounded-lg hover:bg-[#00B8E6] transition-colors font-medium"
          >
            <Plus className="w-4 h-4" />
            New Task
          </button>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 overflow-hidden">
        {viewMode === "kanban" && (
          <DndContext sensors={sensors} onDragStart={handleDragStart} onDragEnd={handleDragEnd}>
            <div className="flex gap-4 p-6 h-full overflow-x-auto">
              {COLUMNS.map((column) => (
                <KanbanColumn
                  key={column.id}
                  column={column}
                  tasks={tasksByStatus[column.id]}
                  expandedTasks={expandedTasks}
                  onToggleExpanded={toggleExpanded}
                  onTaskClick={setSelectedTask}
                  onCreateTask={(status) => {
                    setCreateDialogStatus(status);
                    setIsCreateDialogOpen(true);
                  }}
                />
              ))}
            </div>

            <DragOverlay>
              {activeTask && (
                <TaskCard
                  task={activeTask}
                  isExpanded={false}
                  onToggleExpanded={() => {}}
                  onClick={() => {}}
                  isDragging
                />
              )}
            </DragOverlay>
          </DndContext>
        )}

        {viewMode === "analytics" && analytics && (
          <AnalyticsView analytics={analytics} />
        )}

        {viewMode === "list" && (
          <div className="p-6">
            <p className="text-gray-400">List view coming soon...</p>
          </div>
        )}

        {viewMode === "calendar" && (
          <div className="p-6">
            <p className="text-gray-400">Calendar view coming soon...</p>
          </div>
        )}
      </div>

      {/* Dialogs */}
      <TaskDialogs
        selectedTask={selectedTask}
        isCreateDialogOpen={isCreateDialogOpen}
        onCloseDetail={() => setSelectedTask(null)}
        onCloseCreate={() => {
          setIsCreateDialogOpen(false);
          setCreateDialogStatus(undefined);
        }}
        onUpdateTask={handleUpdateTask}
        onDeleteTask={handleDeleteTask}
        onCreateTask={handleCreateTask}
        onAddComment={handleAddComment}
        onAddTimeEntry={handleAddTimeEntry}
      />
    </div>
  );
}

// Kanban Column Component
function KanbanColumn({
  column,
  tasks,
  expandedTasks,
  onToggleExpanded,
  onTaskClick,
  onCreateTask,
}: {
  column: { id: TaskStatus; title: string; color: string };
  tasks: Task[];
  expandedTasks: Set<number>;
  onToggleExpanded: (taskId: number) => void;
  onTaskClick: (task: Task) => void;
  onCreateTask: (status: TaskStatus) => void;
}) {
  return (
    <div className="flex-shrink-0 w-80 flex flex-col bg-white/5 rounded-lg">
      {/* Column Header */}
      <div className="flex items-center justify-between p-4 border-b border-white/10">
        <div className="flex items-center gap-2">
          <div className={`w-3 h-3 rounded-full ${column.color}`} />
          <h3 className="font-semibold">{column.title}</h3>
          <span className="text-sm text-gray-400">({tasks.length})</span>
        </div>
        <button
          onClick={() => onCreateTask(column.id)}
          className="p-1 hover:bg-white/10 rounded transition-colors"
        >
          <Plus className="w-4 h-4" />
        </button>
      </div>

      {/* Tasks */}
      <div className="flex-1 overflow-y-auto p-4 space-y-3">
        <AnimatePresence>
          {tasks.map((task) => (
            <div key={task.id}>
              <TaskCard
                task={task}
                isExpanded={expandedTasks.has(task.id)}
                onToggleExpanded={onToggleExpanded}
                onClick={onTaskClick}
              />
              
              {/* Subtasks */}
              {expandedTasks.has(task.id) && task.subtasks && task.subtasks.length > 0 && (
                <div className="ml-4 mt-2 space-y-2 border-l-2 border-white/10 pl-3">
                  {task.subtasks.map((subtask) => (
                    <TaskCard
                      key={subtask.id}
                      task={subtask}
                      isExpanded={false}
                      onToggleExpanded={() => {}}
                      onClick={onTaskClick}
                      isSubtask
                    />
                  ))}
                </div>
              )}
            </div>
          ))}
        </AnimatePresence>
      </div>
    </div>
  );
}

// Task Card Component
function TaskCard({
  task,
  isExpanded,
  onToggleExpanded,
  onClick,
  isDragging = false,
  isSubtask = false,
}: {
  task: Task;
  isExpanded: boolean;
  onToggleExpanded: (taskId: number) => void;
  onClick: (task: Task) => void;
  isDragging?: boolean;
  isSubtask?: boolean;
}) {
  const hasSubtasks = task.subtasks && task.subtasks.length > 0;

  return (
    <motion.div
      layout
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      exit={{ opacity: 0, y: -20 }}
      className={`
        p-4 bg-white/5 border border-white/10 rounded-lg cursor-pointer
        hover:border-[#00D4FF] transition-all group
        ${isDragging ? "opacity-50 rotate-2" : ""}
        ${isSubtask ? "text-sm" : ""}
        ${task.sharedTaskId ? "border-l-4 border-l-purple-500" : ""}
      `}
      onClick={() => onClick(task)}
    >
      {/* Header */}
      <div className="flex items-start justify-between gap-2 mb-2">
        <div className="flex items-center gap-2 flex-1">
          {hasSubtasks && (
            <button
              onClick={(e) => {
                e.stopPropagation();
                onToggleExpanded(task.id);
              }}
              className="hover:bg-white/10 rounded p-0.5"
            >
              {isExpanded ? (
                <ChevronDown className="w-4 h-4" />
              ) : (
                <ChevronRight className="w-4 h-4" />
              )}
            </button>
          )}
          <h4 className="font-medium flex-1">{task.title}</h4>
        </div>
        
        <div className="flex items-center gap-1">
          {task.starred && <Star className="w-4 h-4 text-yellow-400 fill-yellow-400" />}
          {task.hasInProgressAttempt && <Loader2 className="w-4 h-4 text-blue-400 animate-spin" />}
          {task.lastAttemptFailed && <XCircle className="w-4 h-4 text-red-400" />}
          {task.parentTaskId && <LinkIcon className="w-4 h-4 text-purple-400" />}
          {task.sharedTaskId && <Users className="w-4 h-4 text-purple-400" />}
        </div>
      </div>

      {/* Description */}
      {task.description && (
        <p className="text-sm text-gray-400 mb-3 line-clamp-2">{task.description}</p>
      )}

      {/* Progress Bar */}
      {task.progress > 0 && (
        <div className="mb-3">
          <div className="flex items-center justify-between text-xs text-gray-400 mb-1">
            <span>Progress</span>
            <span>{task.progress}%</span>
          </div>
          <div className="h-1.5 bg-white/10 rounded-full overflow-hidden">
            <div
              className="h-full bg-[#00D4FF] transition-all"
              style={{ width: `${task.progress}%` }}
            />
          </div>
        </div>
      )}

      {/* Tags */}
      {task.tags && task.tags.length > 0 && (
        <div className="flex flex-wrap gap-1 mb-3">
          {task.tags.map((tag, index) => (
            <span
              key={index}
              className="px-2 py-0.5 text-xs bg-white/10 rounded-full"
            >
              {tag}
            </span>
          ))}
        </div>
      )}

      {/* Footer */}
      <div className="flex items-center justify-between text-xs text-gray-400">
        <div className="flex items-center gap-3">
          <span className={`px-2 py-1 border rounded ${PRIORITY_COLORS[task.priority]}`}>
            {task.priority}
          </span>
          
          {task.dueDate && (
            <div className="flex items-center gap-1">
              <Clock className="w-3 h-3" />
              {new Date(task.dueDate).toLocaleDateString()}
            </div>
          )}
          
          {task.assignee && (
            <div className="flex items-center gap-1">
              <Users className="w-3 h-3" />
              {task.assignee}
            </div>
          )}
        </div>

        {hasSubtasks && (
          <span className="text-purple-400">
            {task.subtasks!.filter(s => s.status === "done").length}/{task.subtasks!.length} subtasks
          </span>
        )}
      </div>
    </motion.div>
  );
}

// Analytics View Component
function AnalyticsView({ analytics }: { analytics: any }) {
  return (
    <div className="p-6 space-y-6">
      <h2 className="text-2xl font-bold">Analytics</h2>

      {/* Key Metrics */}
      <div className="grid grid-cols-4 gap-4">
        <div className="p-6 bg-white/5 rounded-lg border border-white/10">
          <div className="text-3xl font-bold text-[#00D4FF]">{analytics.total}</div>
          <div className="text-sm text-gray-400 mt-1">Total Tasks</div>
        </div>
        
        <div className="p-6 bg-white/5 rounded-lg border border-white/10">
          <div className="text-3xl font-bold text-green-400">{analytics.completed}</div>
          <div className="text-sm text-gray-400 mt-1">Completed</div>
        </div>
        
        <div className="p-6 bg-white/5 rounded-lg border border-white/10">
          <div className="text-3xl font-bold text-blue-400">{analytics.inProgress}</div>
          <div className="text-sm text-gray-400 mt-1">In Progress</div>
        </div>
        
        <div className="p-6 bg-white/5 rounded-lg border border-white/10">
          <div className="text-3xl font-bold text-purple-400">{analytics.completionRate}%</div>
          <div className="text-sm text-gray-400 mt-1">Completion Rate</div>
        </div>
      </div>

      {/* Priority Breakdown */}
      <div className="p-6 bg-white/5 rounded-lg border border-white/10">
        <h3 className="text-lg font-semibold mb-4">Priority Breakdown</h3>
        <div className="space-y-3">
          {Object.entries(analytics.priorityBreakdown).map(([priority, count]) => (
            <div key={priority} className="flex items-center gap-3">
              <div className="w-24 text-sm capitalize">{priority}</div>
              <div className="flex-1 h-8 bg-white/10 rounded-full overflow-hidden">
                <div
                  className={`h-full ${
                    priority === "urgent" ? "bg-red-400" :
                    priority === "high" ? "bg-orange-400" :
                    priority === "medium" ? "bg-blue-400" :
                    "bg-gray-400"
                  }`}
                  style={{ width: `${(count as number / analytics.total) * 100}%` }}
                />
              </div>
              <div className="w-12 text-right text-sm">{count as number}</div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
