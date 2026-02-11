import { useState, useEffect, useMemo } from "react";
import { DndContext, DragEndEvent, DragOverlay, DragStartEvent, PointerSensor, useSensor, useSensors } from "@dnd-kit/core";
import { SortableContext, verticalListSortingStrategy } from "@dnd-kit/sortable";
import { motion, AnimatePresence } from "framer-motion";
import { Search, Plus, Filter, SortAsc, Grid, List, Calendar, BarChart3, Settings, X, ChevronDown } from "lucide-react";
import { trpc } from "@/lib/trpc";
import { useAuth } from "@/_core/hooks/useAuth";

// Task types with all vibe-kanban features
export type TaskStatus = "backlog" | "todo" | "in_progress" | "review" | "done" | "cancelled";
export type TaskPriority = "low" | "medium" | "high" | "urgent";
export type ViewMode = "kanban" | "list" | "calendar" | "analytics";
export type SortMode = "manual" | "priority" | "due_date" | "created" | "updated";

export interface Task {
  id: number;
  taskId: string; // Human-readable ID like "TASK-1"
  userId: number;
  title: string;
  description?: string;
  status: TaskStatus;
  priority: TaskPriority;
  assignee?: string;
  dueDate?: Date;
  tags: string[];
  estimatedTime?: number;
  progress: number;
  starred: boolean;
  parentTaskId?: number;
  sharedTaskId?: string;
  projectId?: string;
  hasInProgressAttempt: boolean;
  lastAttemptFailed: boolean;
  executor?: string;
  isRecurring: boolean;
  recurrencePattern?: string;
  recurrenceEndDate?: Date;
  parentRecurringTaskId?: number;
  createdAt: Date;
  updatedAt: Date;
}

// Column configuration with WIP limits
interface ColumnConfig {
  id: TaskStatus;
  title: string;
  color: string;
  hidden: boolean;
  wipLimit?: number;
  sortOrder: number;
}

const defaultColumns: ColumnConfig[] = [
  { id: "backlog", title: "Backlog", color: "#6B7280", hidden: true, sortOrder: 0 },
  { id: "todo", title: "To Do", color: "#3B82F6", hidden: false, wipLimit: 10, sortOrder: 1 },
  { id: "in_progress", title: "In Progress", color: "#F59E0B", hidden: false, wipLimit: 5, sortOrder: 2 },
  { id: "review", title: "Review", color: "#8B5CF6", hidden: false, wipLimit: 3, sortOrder: 3 },
  { id: "done", title: "Done", color: "#10B981", hidden: false, sortOrder: 4 },
  { id: "cancelled", title: "Cancelled", color: "#EF4444", hidden: true, sortOrder: 5 },
];

export default function TasksEnhancedFinal() {
  const { user } = useAuth();
  const [viewMode, setViewMode] = useState<ViewMode>("kanban");
  const [sortMode, setSortMode] = useState<SortMode>("manual");
  const [searchQuery, setSearchQuery] = useState("");
  const [priorityFilter, setPriorityFilter] = useState<TaskPriority | "all">("all");
  const [activeTab, setActiveTab] = useState<"active" | "all" | TaskStatus>("active");
  const [columns, setColumns] = useState<ColumnConfig[]>(defaultColumns);
  const [showColumnSettings, setShowColumnSettings] = useState(false);
  const [activeTaskId, setActiveTaskId] = useState<number | null>(null);
  const [showTaskPanel, setShowTaskPanel] = useState(false);

  // Fetch tasks from API
  const { data: tasks = [], isLoading, refetch } = trpc.tasks.list.useQuery();
  
  // Mutations
  const updateTaskMutation = trpc.tasks.update.useMutation({
    onSuccess: () => refetch(),
  });

  const createTaskMutation = trpc.tasks.create.useMutation({
    onSuccess: () => refetch(),
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
    let filtered = tasks;

    // Search filter
    if (searchQuery) {
      const query = searchQuery.toLowerCase();
      filtered = filtered.filter(
        (task) =>
          task.title.toLowerCase().includes(query) ||
          task.description?.toLowerCase().includes(query) ||
          task.taskId.toLowerCase().includes(query) ||
          task.tags.some((tag) => tag.toLowerCase().includes(query))
      );
    }

    // Priority filter
    if (priorityFilter !== "all") {
      filtered = filtered.filter((task) => task.priority === priorityFilter);
    }

    // Tab filter
    if (activeTab === "active") {
      // Show only visible columns
      const visibleStatuses = columns.filter((col) => !col.hidden).map((col) => col.id);
      filtered = filtered.filter((task) => visibleStatuses.includes(task.status));
    } else if (activeTab !== "all") {
      // Show specific status
      filtered = filtered.filter((task) => task.status === activeTab);
    }

    // Sort
    if (sortMode !== "manual") {
      filtered = [...filtered].sort((a, b) => {
        switch (sortMode) {
          case "priority":
            const priorityOrder = { urgent: 0, high: 1, medium: 2, low: 3 };
            return priorityOrder[a.priority] - priorityOrder[b.priority];
          case "due_date":
            if (!a.dueDate) return 1;
            if (!b.dueDate) return -1;
            return new Date(a.dueDate).getTime() - new Date(b.dueDate).getTime();
          case "created":
            return new Date(b.createdAt).getTime() - new Date(a.createdAt).getTime();
          case "updated":
            return new Date(b.updatedAt).getTime() - new Date(a.updatedAt).getTime();
          default:
            return 0;
        }
      });
    }

    return filtered;
  }, [tasks, searchQuery, priorityFilter, activeTab, columns, sortMode]);

  // Group tasks by status
  const tasksByStatus = useMemo(() => {
    const grouped: Record<TaskStatus, Task[]> = {
      backlog: [],
      todo: [],
      in_progress: [],
      review: [],
      done: [],
      cancelled: [],
    };

    filteredTasks.forEach((task) => {
      grouped[task.status].push(task);
    });

    return grouped;
  }, [filteredTasks]);

  // Handle drag end
  const handleDragEnd = (event: DragEndEvent) => {
    const { active, over } = event;
    
    if (!over || sortMode !== "manual") return;

    const taskId = active.id as number;
    const newStatus = over.id as TaskStatus;

    const task = tasks.find((t) => t.id === taskId);
    if (!task || task.status === newStatus) return;

    // Check WIP limit
    const targetColumn = columns.find((col) => col.id === newStatus);
    if (targetColumn?.wipLimit) {
      const currentCount = tasksByStatus[newStatus].length;
      if (currentCount >= targetColumn.wipLimit) {
        alert(`WIP limit reached for ${targetColumn.title} (${targetColumn.wipLimit})`);
        return;
      }
    }

    // Update task status
    updateTaskMutation.mutate({
      id: taskId,
      status: newStatus,
    });
  };

  // Create task in specific column
  const handleCreateTask = (status: TaskStatus) => {
    const taskNumber = tasks.length + 1;
    createTaskMutation.mutate({
      taskId: `TASK-${taskNumber}`,
      userId: user?.id || 1,
      title: "New Task",
      description: "",
      status,
      priority: "medium",
      tags: [],
      progress: 0,
      starred: false,
      hasInProgressAttempt: false,
      lastAttemptFailed: false,
      isRecurring: false,
    });
  };

  // Toggle column visibility
  const toggleColumnVisibility = (columnId: TaskStatus) => {
    setColumns((prev) =>
      prev.map((col) =>
        col.id === columnId ? { ...col, hidden: !col.hidden } : col
      )
    );
  };

  // Get visible columns
  const visibleColumns = useMemo(() => {
    return columns.filter((col) => !col.hidden).sort((a, b) => a.sortOrder - b.sortOrder);
  }, [columns]);

  // Get hidden status tabs
  const hiddenStatusTabs = useMemo(() => {
    return columns.filter((col) => col.hidden && tasksByStatus[col.id].length > 0);
  }, [columns, tasksByStatus]);

  if (isLoading) {
    return (
      <div className="flex items-center justify-center h-full">
        <div className="text-gray-400">Loading tasks...</div>
      </div>
    );
  }

  return (
    <div className="flex flex-col h-full bg-[#0A0A0A] text-white">
      {/* Header with tabs and controls */}
      <div className="border-b border-gray-800">
        {/* Status Tabs */}
        <div className="flex items-center gap-2 px-6 pt-4">
          <button
            onClick={() => setActiveTab("active")}
            className={`px-4 py-2 rounded-t-lg transition-colors ${
              activeTab === "active"
                ? "bg-gray-800 text-white"
                : "text-gray-400 hover:text-white hover:bg-gray-900"
            }`}
          >
            Active Kanban
          </button>
          <button
            onClick={() => setActiveTab("all")}
            className={`px-4 py-2 rounded-t-lg transition-colors ${
              activeTab === "all"
                ? "bg-gray-800 text-white"
                : "text-gray-400 hover:text-white hover:bg-gray-900"
            }`}
          >
            All
          </button>
          {hiddenStatusTabs.map((col) => (
            <button
              key={col.id}
              onClick={() => setActiveTab(col.id)}
              className={`px-4 py-2 rounded-t-lg transition-colors ${
                activeTab === col.id
                  ? "bg-gray-800 text-white"
                  : "text-gray-400 hover:text-white hover:bg-gray-900"
              }`}
            >
              {col.title} ({tasksByStatus[col.id].length})
            </button>
          ))}
        </div>

        {/* Filter Bar */}
        <div className="flex items-center gap-4 px-6 py-4">
          {/* Search */}
          <div className="flex-1 relative">
            <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-gray-500" />
            <input
              type="text"
              placeholder="Search tasks..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              className="w-full pl-10 pr-4 py-2 bg-gray-900 border border-gray-800 rounded-lg text-white placeholder-gray-500 focus:outline-none focus:border-[#00D4FF]"
            />
          </div>

          {/* Priority Filter */}
          <select
            value={priorityFilter}
            onChange={(e) => setPriorityFilter(e.target.value as TaskPriority | "all")}
            className="px-4 py-2 bg-gray-900 border border-gray-800 rounded-lg text-white focus:outline-none focus:border-[#00D4FF]"
          >
            <option value="all">All Priorities</option>
            <option value="urgent">Urgent</option>
            <option value="high">High</option>
            <option value="medium">Medium</option>
            <option value="low">Low</option>
          </select>

          {/* Sort Mode */}
          <select
            value={sortMode}
            onChange={(e) => setSortMode(e.target.value as SortMode)}
            className="px-4 py-2 bg-gray-900 border border-gray-800 rounded-lg text-white focus:outline-none focus:border-[#00D4FF]"
          >
            <option value="manual">Manual</option>
            <option value="priority">Priority</option>
            <option value="due_date">Due Date</option>
            <option value="created">Created</option>
            <option value="updated">Updated</option>
          </select>

          {/* View Mode */}
          <div className="flex items-center gap-2 bg-gray-900 border border-gray-800 rounded-lg p-1">
            <button
              onClick={() => setViewMode("kanban")}
              className={`p-2 rounded ${
                viewMode === "kanban" ? "bg-[#00D4FF] text-black" : "text-gray-400 hover:text-white"
              }`}
            >
              <Grid className="w-4 h-4" />
            </button>
            <button
              onClick={() => setViewMode("list")}
              className={`p-2 rounded ${
                viewMode === "list" ? "bg-[#00D4FF] text-black" : "text-gray-400 hover:text-white"
              }`}
            >
              <List className="w-4 h-4" />
            </button>
            <button
              onClick={() => setViewMode("calendar")}
              className={`p-2 rounded ${
                viewMode === "calendar" ? "bg-[#00D4FF] text-black" : "text-gray-400 hover:text-white"
              }`}
            >
              <Calendar className="w-4 h-4" />
            </button>
            <button
              onClick={() => setViewMode("analytics")}
              className={`p-2 rounded ${
                viewMode === "analytics" ? "bg-[#00D4FF] text-black" : "text-gray-400 hover:text-white"
              }`}
            >
              <BarChart3 className="w-4 h-4" />
            </button>
          </div>

          {/* Column Settings */}
          <button
            onClick={() => setShowColumnSettings(!showColumnSettings)}
            className="p-2 bg-gray-900 border border-gray-800 rounded-lg text-gray-400 hover:text-white"
          >
            <Settings className="w-4 h-4" />
          </button>
        </div>

        {/* Active Filters Chips */}
        {(searchQuery || priorityFilter !== "all" || sortMode !== "manual") && (
          <div className="flex items-center gap-2 px-6 pb-4">
            {searchQuery && (
              <div className="flex items-center gap-2 px-3 py-1 bg-gray-800 rounded-full text-sm">
                <span>Search: {searchQuery}</span>
                <button onClick={() => setSearchQuery("")} className="hover:text-[#00D4FF]">
                  <X className="w-3 h-3" />
                </button>
              </div>
            )}
            {priorityFilter !== "all" && (
              <div className="flex items-center gap-2 px-3 py-1 bg-gray-800 rounded-full text-sm">
                <span>Priority: {priorityFilter}</span>
                <button onClick={() => setPriorityFilter("all")} className="hover:text-[#00D4FF]">
                  <X className="w-3 h-3" />
                </button>
              </div>
            )}
            {sortMode !== "manual" && (
              <div className="flex items-center gap-2 px-3 py-1 bg-gray-800 rounded-full text-sm">
                <span>Sort: {sortMode.replace("_", " ")}</span>
                <button onClick={() => setSortMode("manual")} className="hover:text-[#00D4FF]">
                  <X className="w-3 h-3" />
                </button>
              </div>
            )}
          </div>
        )}
      </div>

      {/* Column Settings Panel */}
      {showColumnSettings && (
        <div className="border-b border-gray-800 bg-gray-900 px-6 py-4">
          <h3 className="text-sm font-semibold mb-3">Column Settings</h3>
          <div className="grid grid-cols-3 gap-4">
            {columns.map((col) => (
              <div key={col.id} className="flex items-center justify-between p-3 bg-gray-800 rounded-lg">
                <div className="flex items-center gap-3">
                  <div
                    className="w-3 h-3 rounded-full"
                    style={{ backgroundColor: col.color }}
                  />
                  <span>{col.title}</span>
                </div>
                <button
                  onClick={() => toggleColumnVisibility(col.id)}
                  className={`px-2 py-1 rounded text-xs ${
                    col.hidden
                      ? "bg-gray-700 text-gray-400"
                      : "bg-[#00D4FF] text-black"
                  }`}
                >
                  {col.hidden ? "Show" : "Hide"}
                </button>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* Main Content */}
      <div className="flex-1 overflow-hidden">
        {viewMode === "kanban" && activeTab === "active" && (
          <KanbanBoard
            columns={visibleColumns}
            tasksByStatus={tasksByStatus}
            onDragEnd={handleDragEnd}
            onCreateTask={handleCreateTask}
            sortMode={sortMode}
            sensors={sensors}
            onTaskClick={(taskId) => {
              setActiveTaskId(taskId);
              setShowTaskPanel(true);
            }}
          />
        )}

        {viewMode === "kanban" && activeTab !== "active" && (
          <ListView
            tasks={filteredTasks}
            onTaskClick={(taskId) => {
              setActiveTaskId(taskId);
              setShowTaskPanel(true);
            }}
          />
        )}

        {viewMode === "list" && (
          <ListView
            tasks={filteredTasks}
            onTaskClick={(taskId) => {
              setActiveTaskId(taskId);
              setShowTaskPanel(true);
            }}
          />
        )}

        {viewMode === "calendar" && (
          <div className="flex items-center justify-center h-full text-gray-400">
            Calendar view coming soon...
          </div>
        )}

        {viewMode === "analytics" && (
          <div className="flex items-center justify-center h-full text-gray-400">
            Analytics view coming soon...
          </div>
        )}
      </div>

      {/* Task Detail Panel (Side Panel instead of Modal) */}
      {showTaskPanel && activeTaskId && (
        <TaskDetailPanel
          taskId={activeTaskId}
          onClose={() => {
            setShowTaskPanel(false);
            setActiveTaskId(null);
          }}
        />
      )}
    </div>
  );
}

// Kanban Board Component
interface KanbanBoardProps {
  columns: ColumnConfig[];
  tasksByStatus: Record<TaskStatus, Task[]>;
  onDragEnd: (event: DragEndEvent) => void;
  onCreateTask: (status: TaskStatus) => void;
  sortMode: SortMode;
  sensors: any;
  onTaskClick: (taskId: number) => void;
}

function KanbanBoard({
  columns,
  tasksByStatus,
  onDragEnd,
  onCreateTask,
  sortMode,
  sensors,
  onTaskClick,
}: KanbanBoardProps) {
  const [activeId, setActiveId] = useState<number | null>(null);

  const handleDragStart = (event: DragStartEvent) => {
    setActiveId(event.active.id as number);
  };

  const handleDragEndInternal = (event: DragEndEvent) => {
    setActiveId(null);
    onDragEnd(event);
  };

  return (
    <DndContext
      sensors={sensors}
      onDragStart={handleDragStart}
      onDragEnd={handleDragEndInternal}
    >
      <div className="flex gap-4 p-6 h-full overflow-x-auto">
        {columns.map((column) => (
          <KanbanColumn
            key={column.id}
            column={column}
            tasks={tasksByStatus[column.id]}
            onCreateTask={() => onCreateTask(column.id)}
            sortMode={sortMode}
            onTaskClick={onTaskClick}
          />
        ))}
      </div>

      <DragOverlay>
        {activeId ? (
          <div className="opacity-50">
            {/* Drag preview */}
          </div>
        ) : null}
      </DragOverlay>
    </DndContext>
  );
}

// Kanban Column Component
interface KanbanColumnProps {
  column: ColumnConfig;
  tasks: Task[];
  onCreateTask: () => void;
  sortMode: SortMode;
  onTaskClick: (taskId: number) => void;
}

function KanbanColumn({ column, tasks, onCreateTask, sortMode, onTaskClick }: KanbanColumnProps) {
  const isWipLimitReached = column.wipLimit && tasks.length >= column.wipLimit;

  return (
    <div className="flex flex-col w-80 flex-shrink-0 bg-gray-900 rounded-lg">
      {/* Column Header */}
      <div className="flex items-center justify-between p-4 border-b border-gray-800">
        <div className="flex items-center gap-3">
          <div
            className="w-3 h-3 rounded-full"
            style={{ backgroundColor: column.color }}
          />
          <h3 className="font-semibold">{column.title}</h3>
          <span className="text-sm text-gray-500">
            {tasks.length}
            {column.wipLimit && ` / ${column.wipLimit}`}
          </span>
          {isWipLimitReached && (
            <span className="text-xs text-red-500 font-semibold">WIP LIMIT</span>
          )}
        </div>
        <button
          onClick={onCreateTask}
          className="p-1 hover:bg-gray-800 rounded transition-colors"
          title="Add task"
        >
          <Plus className="w-4 h-4" />
        </button>
      </div>

      {/* Tasks List */}
      <SortableContext
        items={tasks.map((t) => t.id)}
        strategy={verticalListSortingStrategy}
        disabled={sortMode !== "manual"}
      >
        <div className="flex-1 overflow-y-auto p-4 space-y-3">
          {tasks.length === 0 ? (
            <div className="flex flex-col items-center justify-center py-12 text-gray-500">
              <p className="text-sm">No tasks</p>
              <button
                onClick={onCreateTask}
                className="mt-2 text-xs text-[#00D4FF] hover:underline"
              >
                Create one
              </button>
            </div>
          ) : (
            tasks.map((task) => (
              <TaskCard
                key={task.id}
                task={task}
                onClick={() => onTaskClick(task.id)}
                isDraggable={sortMode === "manual"}
              />
            ))
          )}
        </div>
      </SortableContext>
    </div>
  );
}

// Task Card Component (simplified - full implementation would be larger)
interface TaskCardProps {
  task: Task;
  onClick: () => void;
  isDraggable: boolean;
}

function TaskCard({ task, onClick, isDraggable }: TaskCardProps) {
  const priorityColors = {
    urgent: "bg-red-500",
    high: "bg-orange-500",
    medium: "bg-yellow-500",
    low: "bg-green-500",
  };

  return (
    <motion.div
      layout
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      exit={{ opacity: 0, y: -20 }}
      onClick={onClick}
      className="p-4 bg-gray-800 rounded-lg cursor-pointer hover:bg-gray-750 transition-colors"
    >
      <div className="flex items-start justify-between mb-2">
        <span className="text-xs text-gray-500">{task.taskId}</span>
        <div className={`w-2 h-2 rounded-full ${priorityColors[task.priority]}`} />
      </div>
      <h4 className="font-medium mb-2">{task.title}</h4>
      {task.description && (
        <p className="text-sm text-gray-400 line-clamp-2 mb-2">{task.description}</p>
      )}
      {task.tags.length > 0 && (
        <div className="flex flex-wrap gap-1">
          {task.tags.map((tag) => (
            <span
              key={tag}
              className="px-2 py-1 text-xs bg-gray-700 rounded"
            >
              {tag}
            </span>
          ))}
        </div>
      )}
    </motion.div>
  );
}

// List View Component (placeholder)
interface ListViewProps {
  tasks: Task[];
  onTaskClick: (taskId: number) => void;
}

function ListView({ tasks, onTaskClick }: ListViewProps) {
  return (
    <div className="p-6">
      <div className="bg-gray-900 rounded-lg overflow-hidden">
        <table className="w-full">
          <thead className="bg-gray-800">
            <tr>
              <th className="px-4 py-3 text-left text-sm font-semibold">ID</th>
              <th className="px-4 py-3 text-left text-sm font-semibold">Title</th>
              <th className="px-4 py-3 text-left text-sm font-semibold">Status</th>
              <th className="px-4 py-3 text-left text-sm font-semibold">Priority</th>
              <th className="px-4 py-3 text-left text-sm font-semibold">Due Date</th>
            </tr>
          </thead>
          <tbody>
            {tasks.map((task) => (
              <tr
                key={task.id}
                onClick={() => onTaskClick(task.id)}
                className="border-t border-gray-800 hover:bg-gray-800 cursor-pointer"
              >
                <td className="px-4 py-3 text-sm">{task.taskId}</td>
                <td className="px-4 py-3">{task.title}</td>
                <td className="px-4 py-3 text-sm">{task.status}</td>
                <td className="px-4 py-3 text-sm">{task.priority}</td>
                <td className="px-4 py-3 text-sm">
                  {task.dueDate ? new Date(task.dueDate).toLocaleDateString() : "-"}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  );
}

// Task Detail Panel Component (placeholder)
interface TaskDetailPanelProps {
  taskId: number;
  onClose: () => void;
}

function TaskDetailPanel({ taskId, onClose }: TaskDetailPanelProps) {
  return (
    <motion.div
      initial={{ x: "100%" }}
      animate={{ x: 0 }}
      exit={{ x: "100%" }}
      className="fixed right-0 top-0 bottom-0 w-[500px] bg-gray-900 border-l border-gray-800 shadow-2xl z-50"
    >
      <div className="flex items-center justify-between p-4 border-b border-gray-800">
        <h2 className="text-lg font-semibold">Task Details</h2>
        <button
          onClick={onClose}
          className="p-2 hover:bg-gray-800 rounded transition-colors"
        >
          <X className="w-5 h-5" />
        </button>
      </div>
      <div className="p-6">
        <p className="text-gray-400">Task ID: {taskId}</p>
        <p className="text-sm text-gray-500 mt-2">Full task details coming soon...</p>
      </div>
    </motion.div>
  );
}
