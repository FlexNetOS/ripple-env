import { useState, useMemo, useCallback } from "react";
import {
  DndContext,
  DragEndEvent,
  DragOverEvent,
  DragOverlay,
  DragStartEvent,
  PointerSensor,
  useSensor,
  useSensors,
  closestCorners,
} from "@dnd-kit/core";
import {
  SortableContext,
  useSortable,
  verticalListSortingStrategy,
} from "@dnd-kit/sortable";
import { CSS } from "@dnd-kit/utilities";
import { motion, AnimatePresence } from "framer-motion";
import {
  Plus,
  Search,
  Filter,
  SortAsc,
  Clock,
  User,
  Tag,
  MoreVertical,
  Edit,
  Trash2,
  Copy,
  Archive,
  Star,
  Calendar,
  AlertCircle,
  CheckCircle2,
  Timer,
  Zap,
  TrendingUp,
  BarChart3,
  Download,
  Upload,
  Settings,
  Sparkles,
} from "lucide-react";
import { TaskDetailDialog, TaskFormDialog } from "@/components/tasks/TaskDialogs";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Textarea } from "@/components/ui/textarea";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Badge } from "@/components/ui/badge";
import { DropdownMenu, DropdownMenuContent, DropdownMenuItem, DropdownMenuSeparator, DropdownMenuTrigger } from "@/components/ui/dropdown-menu";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Calendar as CalendarComponent } from "@/components/ui/calendar";
import { Popover, PopoverContent, PopoverTrigger } from "@/components/ui/popover";
import { Progress } from "@/components/ui/progress";
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar";
import { Separator } from "@/components/ui/separator";
import { ScrollArea } from "@/components/ui/scroll-area";

type TaskStatus = "todo" | "in_progress" | "review" | "done" | "cancelled";
type TaskPriority = "low" | "medium" | "high" | "urgent";
type ViewMode = "kanban" | "list" | "calendar" | "analytics";

interface Comment {
  id: string;
  author: string;
  content: string;
  timestamp: Date;
  avatar?: string;
}

interface Activity {
  id: string;
  type: "created" | "updated" | "commented" | "status_changed" | "assigned";
  user: string;
  description: string;
  timestamp: Date;
}

interface TimeEntry {
  id: string;
  duration: number; // in minutes
  description: string;
  timestamp: Date;
}

interface Task {
  id: string;
  title: string;
  description: string;
  status: TaskStatus;
  priority: TaskPriority;
  assignee?: string;
  dueDate?: Date;
  tags: string[];
  comments: Comment[];
  activities: Activity[];
  timeEntries: TimeEntry[];
  estimatedTime?: number; // in minutes
  // Vibe-kanban inspired features
  parent_task_id?: string; // For hierarchical tasks/subtasks
  subtasks?: Task[]; // Child tasks
  shared_task_id?: string; // For shared/collaborative tasks
  project_id?: string; // Multi-project support
  has_in_progress_attempt?: boolean; // Real-time execution status
  last_attempt_failed?: boolean; // Failure tracking
  executor?: string; // AI agent or user
  progress: number; // 0-100
  dependencies: string[]; // task IDs
  starred: boolean;
  createdAt: Date;
  updatedAt: Date;
}

// Mock data with enhanced fields
const mockTasks: Task[] = [
  {
    id: "1",
    title: "Design new landing page",
    description: "Create wireframes and mockups for the new landing page with modern design principles",
    status: "in_progress",
    priority: "high",
    assignee: "Demo User",
    dueDate: new Date("2026-02-15"),
    tags: ["design", "ui/ux", "frontend"],
    comments: [
      {
        id: "c1",
        author: "Demo User",
        content: "Started working on the wireframes",
        timestamp: new Date("2026-02-10T10:00:00"),
      },
    ],
    activities: [
      {
        id: "a1",
        type: "created",
        user: "Demo User",
        description: "created this task",
        timestamp: new Date("2026-02-09T09:00:00"),
      },
      {
        id: "a2",
        type: "status_changed",
        user: "Demo User",
        description: "moved from To Do to In Progress",
        timestamp: new Date("2026-02-10T09:30:00"),
      },
    ],
    timeEntries: [
      {
        id: "t1",
        duration: 120,
        description: "Initial wireframe sketches",
        timestamp: new Date("2026-02-10T10:00:00"),
      },
    ],
    estimatedTime: 480,
    progress: 40,
    dependencies: [],
    starred: true,
    createdAt: new Date("2026-02-09T09:00:00"),
    updatedAt: new Date("2026-02-10T10:00:00"),
  },
  {
    id: "2",
    title: "Implement authentication",
    description: "Add OAuth support for Google and GitHub with secure token management",
    status: "todo",
    priority: "urgent",
    assignee: undefined,
    dueDate: new Date("2026-02-12"),
    tags: ["backend", "security", "auth"],
    comments: [],
    activities: [
      {
        id: "a3",
        type: "created",
        user: "Admin",
        description: "created this task",
        timestamp: new Date("2026-02-08T14:00:00"),
      },
    ],
    timeEntries: [],
    estimatedTime: 360,
    progress: 0,
    dependencies: [],
    starred: false,
    createdAt: new Date("2026-02-08T14:00:00"),
    updatedAt: new Date("2026-02-08T14:00:00"),
  },
  {
    id: "3",
    title: "Write API documentation",
    description: "Document all REST API endpoints with examples and error codes",
    status: "todo",
    priority: "medium",
    assignee: undefined,
    dueDate: new Date("2026-02-20"),
    tags: ["docs", "api"],
    comments: [],
    activities: [
      {
        id: "a4",
        type: "created",
        user: "Demo User",
        description: "created this task",
        timestamp: new Date("2026-02-07T11:00:00"),
      },
    ],
    timeEntries: [],
    estimatedTime: 240,
    progress: 0,
    dependencies: ["2"],
    starred: false,
    createdAt: new Date("2026-02-07T11:00:00"),
    updatedAt: new Date("2026-02-07T11:00:00"),
  },
  {
    id: "4",
    title: "Fix mobile responsiveness",
    description: "Ensure all pages work seamlessly on mobile devices and tablets",
    status: "in_progress",
    priority: "high",
    assignee: "Demo User",
    dueDate: new Date("2026-02-14"),
    tags: ["frontend", "mobile", "css"],
    comments: [
      {
        id: "c2",
        author: "Demo User",
        content: "Testing on various devices",
        timestamp: new Date("2026-02-11T15:00:00"),
      },
    ],
    activities: [
      {
        id: "a5",
        type: "created",
        user: "Demo User",
        description: "created this task",
        timestamp: new Date("2026-02-06T10:00:00"),
      },
      {
        id: "a6",
        type: "status_changed",
        user: "Demo User",
        description: "moved from To Do to In Progress",
        timestamp: new Date("2026-02-11T14:00:00"),
      },
    ],
    timeEntries: [
      {
        id: "t2",
        duration: 180,
        description: "Mobile layout fixes",
        timestamp: new Date("2026-02-11T14:00:00"),
      },
    ],
    estimatedTime: 300,
    progress: 60,
    dependencies: [],
    starred: false,
    createdAt: new Date("2026-02-06T10:00:00"),
    updatedAt: new Date("2026-02-11T15:00:00"),
  },
  {
    id: "5",
    title: "Set up CI/CD pipeline",
    description: "Configure GitHub Actions for automated testing and deployment to production",
    status: "review",
    priority: "high",
    assignee: "DevOps Team",
    dueDate: new Date("2026-02-13"),
    tags: ["devops", "automation", "ci/cd"],
    comments: [
      {
        id: "c3",
        author: "DevOps Team",
        content: "Pipeline configured, needs review",
        timestamp: new Date("2026-02-11T16:00:00"),
      },
    ],
    activities: [
      {
        id: "a7",
        type: "created",
        user: "Admin",
        description: "created this task",
        timestamp: new Date("2026-02-05T09:00:00"),
      },
      {
        id: "a8",
        type: "status_changed",
        user: "DevOps Team",
        description: "moved from In Progress to Review",
        timestamp: new Date("2026-02-11T16:00:00"),
      },
    ],
    timeEntries: [
      {
        id: "t3",
        duration: 240,
        description: "GitHub Actions setup",
        timestamp: new Date("2026-02-11T12:00:00"),
      },
    ],
    estimatedTime: 300,
    progress: 90,
    dependencies: [],
    starred: true,
    createdAt: new Date("2026-02-05T09:00:00"),
    updatedAt: new Date("2026-02-11T16:00:00"),
  },
  {
    id: "6",
    title: "Database optimization",
    description: "Add indexes and optimize slow queries for better performance",
    status: "done",
    priority: "medium",
    assignee: "Backend Team",
    dueDate: new Date("2026-02-10"),
    tags: ["backend", "database", "performance"],
    comments: [
      {
        id: "c4",
        author: "Backend Team",
        content: "All queries optimized and tested",
        timestamp: new Date("2026-02-10T17:00:00"),
      },
    ],
    activities: [
      {
        id: "a9",
        type: "created",
        user: "Admin",
        description: "created this task",
        timestamp: new Date("2026-02-04T10:00:00"),
      },
      {
        id: "a10",
        type: "status_changed",
        user: "Backend Team",
        description: "moved from Review to Done",
        timestamp: new Date("2026-02-10T17:00:00"),
      },
    ],
    timeEntries: [
      {
        id: "t4",
        duration: 360,
        description: "Query optimization and indexing",
        timestamp: new Date("2026-02-09T10:00:00"),
      },
    ],
    estimatedTime: 360,
    progress: 100,
    dependencies: [],
    starred: false,
    createdAt: new Date("2026-02-04T10:00:00"),
    updatedAt: new Date("2026-02-10T17:00:00"),
  },
];

const statusConfig: Record<
  TaskStatus,
  { label: string; color: string; bgColor: string; icon: React.ReactNode }
> = {
  todo: {
    label: "To Do",
    color: "text-gray-400",
    bgColor: "bg-gray-500/10",
    icon: <AlertCircle className="w-4 h-4" />,
  },
  in_progress: {
    label: "In Progress",
    color: "text-[#00D4FF]",
    bgColor: "bg-[#00D4FF]/10",
    icon: <Timer className="w-4 h-4" />,
  },
  review: {
    label: "Review",
    color: "text-[#9B7BFF]",
    bgColor: "bg-[#9B7BFF]/10",
    icon: <Zap className="w-4 h-4" />,
  },
  done: {
    label: "Done",
    color: "text-[#00E676]",
    bgColor: "bg-[#00E676]/10",
    icon: <CheckCircle2 className="w-4 h-4" />,
  },
};

const priorityConfig: Record<
  TaskPriority,
  { label: string; color: string; bgColor: string }
> = {
  low: { label: "Low", color: "text-gray-400", bgColor: "bg-gray-500/20" },
  medium: { label: "Medium", color: "text-[#00D4FF]", bgColor: "bg-[#00D4FF]/20" },
  high: { label: "High", color: "text-orange-400", bgColor: "bg-orange-500/20" },
  urgent: { label: "Urgent", color: "text-red-400", bgColor: "bg-red-500/20" },
};

// Sortable Task Card Component
function SortableTaskCard({
  task,
  onViewDetails,
  onQuickEdit,
  onDelete,
}: {
  task: Task;
  onViewDetails: (task: Task) => void;
  onQuickEdit: (task: Task) => void;
  onDelete: (taskId: string) => void;
}) {
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

  const totalTimeSpent = task.timeEntries.reduce((acc, entry) => acc + entry.duration, 0);
  const timeProgress = task.estimatedTime
    ? Math.min((totalTimeSpent / task.estimatedTime) * 100, 100)
    : 0;

  return (
    <div ref={setNodeRef} style={style} {...attributes} {...listeners}>
      <motion.div
        layout
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        exit={{ opacity: 0, y: -20 }}
        className="group relative bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-4 hover:border-[#00D4FF]/30 transition-all cursor-pointer"
        onClick={() => onViewDetails(task)}
      >
        {/* Priority indicator */}
        <div
          className={`absolute top-0 left-0 w-1 h-full rounded-l-lg ${
            task.priority === "urgent"
              ? "bg-red-500"
              : task.priority === "high"
              ? "bg-orange-500"
              : task.priority === "medium"
              ? "bg-[#00D4FF]"
              : "bg-gray-500"
          } opacity-0 group-hover:opacity-100 transition-opacity`}
        />

        <div className="space-y-3">
          {/* Header */}
          <div className="flex items-start justify-between gap-2">
            <div className="flex-1 flex items-start gap-2">
              {task.starred && <Star className="w-4 h-4 text-yellow-400 fill-yellow-400 flex-shrink-0 mt-0.5" />}
              <h3 className="font-medium text-white text-sm line-clamp-2 flex-1">
                {task.title}
              </h3>
            </div>
            <div className="flex items-center gap-1">
              <Badge className={`px-2 py-0.5 text-xs ${priorityConfig[task.priority].bgColor} ${priorityConfig[task.priority].color}`}>
                {priorityConfig[task.priority].label}
              </Badge>
              <DropdownMenu>
                <DropdownMenuTrigger asChild onClick={(e) => e.stopPropagation()}>
                  <Button variant="ghost" size="sm" className="h-6 w-6 p-0">
                    <MoreVertical className="w-4 h-4" />
                  </Button>
                </DropdownMenuTrigger>
                <DropdownMenuContent align="end">
                  <DropdownMenuItem onClick={(e) => { e.stopPropagation(); onQuickEdit(task); }}>
                    <Edit className="w-4 h-4 mr-2" />
                    Edit
                  </DropdownMenuItem>
                  <DropdownMenuItem onClick={(e) => { e.stopPropagation(); }}>
                    <Copy className="w-4 h-4 mr-2" />
                    Duplicate
                  </DropdownMenuItem>
                  <DropdownMenuItem onClick={(e) => { e.stopPropagation(); }}>
                    <Archive className="w-4 h-4 mr-2" />
                    Archive
                  </DropdownMenuItem>
                  <DropdownMenuSeparator />
                  <DropdownMenuItem
                    onClick={(e) => { e.stopPropagation(); onDelete(task.id); }}
                    className="text-red-400"
                  >
                    <Trash2 className="w-4 h-4 mr-2" />
                    Delete
                  </DropdownMenuItem>
                </DropdownMenuContent>
              </DropdownMenu>
            </div>
          </div>

          {/* Description */}
          <p className="text-sm text-gray-400 line-clamp-2">{task.description}</p>

          {/* Progress bar */}
          {task.progress > 0 && (
            <div className="space-y-1">
              <div className="flex items-center justify-between text-xs text-gray-500">
                <span>Progress</span>
                <span>{task.progress}%</span>
              </div>
              <Progress value={task.progress} className="h-1" />
            </div>
          )}

          {/* Tags */}
          {task.tags.length > 0 && (
            <div className="flex flex-wrap gap-1.5">
              {task.tags.slice(0, 3).map((tag) => (
                <Badge key={tag} variant="secondary" className="px-2 py-0.5 text-xs">
                  {tag}
                </Badge>
              ))}
              {task.tags.length > 3 && (
                <Badge variant="secondary" className="px-2 py-0.5 text-xs">
                  +{task.tags.length - 3}
                </Badge>
              )}
            </div>
          )}

          {/* Footer */}
          <div className="flex items-center justify-between text-xs text-gray-500">
            <div className="flex items-center gap-3">
              {task.assignee && (
                <div className="flex items-center gap-1">
                  <User className="w-3 h-3" />
                  <span>{task.assignee}</span>
                </div>
              )}
              {task.comments.length > 0 && (
                <div className="flex items-center gap-1">
                  <span>{task.comments.length} comments</span>
                </div>
              )}
            </div>
            {task.dueDate && (
              <div className="flex items-center gap-1">
                <Calendar className="w-3 h-3" />
                <span>{task.dueDate.toLocaleDateString()}</span>
              </div>
            )}
          </div>

          {/* Time tracking */}
          {task.estimatedTime && (
            <div className="flex items-center gap-2 text-xs text-gray-500">
              <Clock className="w-3 h-3" />
              <span>
                {Math.floor(totalTimeSpent / 60)}h {totalTimeSpent % 60}m / {Math.floor(task.estimatedTime / 60)}h
              </span>
              <div className="flex-1 bg-[#2A2A2A] rounded-full h-1">
                <div
                  className="bg-[#00D4FF] h-1 rounded-full transition-all"
                  style={{ width: `${timeProgress}%` }}
                />
              </div>
            </div>
          )}
        </div>
      </motion.div>
    </div>
  );
}

// Continue in Part 2...

// Kanban Column Component
function KanbanColumn({
  status,
  tasks,
  onViewDetails,
  onQuickEdit,
  onDelete,
  onAddTask,
}: {
  status: TaskStatus;
  tasks: Task[];
  onViewDetails: (task: Task) => void;
  onQuickEdit: (task: Task) => void;
  onDelete: (taskId: string) => void;
  onAddTask: (status: TaskStatus) => void;
}) {
  const config = statusConfig[status];

  return (
    <div className="flex flex-col h-full min-w-[340px]">
      {/* Column Header */}
      <div className="flex items-center justify-between mb-4 pb-3 border-b border-[#2A2A2A]">
        <div className="flex items-center gap-2">
          <div className={`${config.bgColor} p-1.5 rounded`}>
            {config.icon}
          </div>
          <h2 className={`font-semibold ${config.color}`}>{config.label}</h2>
          <Badge variant="secondary" className="px-2 py-0.5 text-xs">
            {tasks.length}
          </Badge>
        </div>
        <Button
          variant="ghost"
          size="sm"
          className="h-8 w-8 p-0"
          onClick={() => onAddTask(status)}
        >
          <Plus className="w-4 h-4" />
        </Button>
      </div>

      {/* Tasks List */}
      <SortableContext items={tasks.map((t) => t.id)} strategy={verticalListSortingStrategy}>
        <ScrollArea className="flex-1 pr-2">
          <div className="space-y-3">
            {tasks.map((task) => (
              <SortableTaskCard
                key={task.id}
                task={task}
                onViewDetails={onViewDetails}
                onQuickEdit={onQuickEdit}
                onDelete={onDelete}
              />
            ))}
          </div>
        </ScrollArea>
      </SortableContext>
    </div>
  );
}

// Main Enhanced Tasks Component
export default function TasksEnhanced() {
  const [tasks, setTasks] = useState<Task[]>(mockTasks);
  const [activeId, setActiveId] = useState<string | null>(null);
  const [searchQuery, setSearchQuery] = useState("");
  const [viewMode, setViewMode] = useState<ViewMode>("kanban");
  const [selectedTask, setSelectedTask] = useState<Task | null>(null);
  const [isTaskDialogOpen, setIsTaskDialogOpen] = useState(false);
  const [isCreateDialogOpen, setIsCreateDialogOpen] = useState(false);
  const [filterPriority, setFilterPriority] = useState<TaskPriority | "all">("all");
  const [filterStatus, setFilterStatus] = useState<TaskStatus | "all">("all");
  const [sortBy, setSortBy] = useState<"dueDate" | "priority" | "created" | "updated">("dueDate");

  const sensors = useSensors(
    useSensor(PointerSensor, {
      activationConstraint: {
        distance: 8,
      },
    })
  );

  const handleDragStart = (event: DragStartEvent) => {
    setActiveId(event.active.id as string);
  };

  const handleDragOver = (event: DragOverEvent) => {
    const { active, over } = event;
    if (!over) return;

    const activeId = active.id as string;
    const overId = over.id as string;

    // Find the task being dragged
    const activeTask = tasks.find((t) => t.id === activeId);
    if (!activeTask) return;

    // Check if we're dragging over a column (status)
    if (["todo", "in_progress", "review", "done"].includes(overId)) {
      const newStatus = overId as TaskStatus;
      if (activeTask.status !== newStatus) {
        setTasks((tasks) =>
          tasks.map((t) =>
            t.id === activeId
              ? {
                  ...t,
                  status: newStatus,
                  updatedAt: new Date(),
                  activities: [
                    ...t.activities,
                    {
                      id: `a${Date.now()}`,
                      type: "status_changed",
                      user: "Demo User",
                      description: `moved from ${statusConfig[t.status].label} to ${statusConfig[newStatus].label}`,
                      timestamp: new Date(),
                    },
                  ],
                }
              : t
          )
        );
      }
    }
  };

  const handleDragEnd = (event: DragEndEvent) => {
    setActiveId(null);
  };

  const handleViewDetails = useCallback((task: Task) => {
    setSelectedTask(task);
    setIsTaskDialogOpen(true);
  }, []);

  const handleQuickEdit = useCallback((task: Task) => {
    setSelectedTask(task);
    setIsCreateDialogOpen(true);
  }, []);

  const handleDelete = useCallback((taskId: string) => {
    setTasks((tasks) => tasks.filter((t) => t.id !== taskId));
  }, []);

  const handleAddTask = useCallback((status: TaskStatus) => {
    setSelectedTask({
      id: `task-${Date.now()}`,
      title: "",
      description: "",
      status,
      priority: "medium",
      tags: [],
      comments: [],
      activities: [],
      timeEntries: [],
      progress: 0,
      dependencies: [],
      starred: false,
      createdAt: new Date(),
      updatedAt: new Date(),
    });
    setIsCreateDialogOpen(true);
  }, []);

  const handleSaveTask = useCallback((task: Task) => {
    setTasks((tasks) => {
      const existing = tasks.find((t) => t.id === task.id);
      if (existing) {
        return tasks.map((t) => (t.id === task.id ? { ...task, updatedAt: new Date() } : t));
      } else {
        return [...tasks, { ...task, createdAt: new Date(), updatedAt: new Date() }];
      }
    });
    setIsCreateDialogOpen(false);
    setSelectedTask(null);
  }, []);

  // Filtering and sorting
  const filteredAndSortedTasks = useMemo(() => {
    let filtered = tasks.filter((task) => {
      const matchesSearch =
        task.title.toLowerCase().includes(searchQuery.toLowerCase()) ||
        task.description.toLowerCase().includes(searchQuery.toLowerCase()) ||
        task.tags.some((tag) => tag.toLowerCase().includes(searchQuery.toLowerCase()));

      const matchesPriority = filterPriority === "all" || task.priority === filterPriority;
      const matchesStatus = filterStatus === "all" || task.status === filterStatus;

      return matchesSearch && matchesPriority && matchesStatus;
    });

    // Sort tasks
    filtered.sort((a, b) => {
      switch (sortBy) {
        case "dueDate":
          if (!a.dueDate) return 1;
          if (!b.dueDate) return -1;
          return a.dueDate.getTime() - b.dueDate.getTime();
        case "priority":
          const priorityOrder = { urgent: 0, high: 1, medium: 2, low: 3 };
          return priorityOrder[a.priority] - priorityOrder[b.priority];
        case "created":
          return b.createdAt.getTime() - a.createdAt.getTime();
        case "updated":
          return b.updatedAt.getTime() - a.updatedAt.getTime();
        default:
          return 0;
      }
    });

    return filtered;
  }, [tasks, searchQuery, filterPriority, filterStatus, sortBy]);

  const tasksByStatus = useMemo(() => {
    return {
      todo: filteredAndSortedTasks.filter((t) => t.status === "todo"),
      in_progress: filteredAndSortedTasks.filter((t) => t.status === "in_progress"),
      review: filteredAndSortedTasks.filter((t) => t.status === "review"),
      done: filteredAndSortedTasks.filter((t) => t.status === "done"),
    };
  }, [filteredAndSortedTasks]);

  // Analytics calculations
  const analytics = useMemo(() => {
    const total = tasks.length;
    const completed = tasks.filter((t) => t.status === "done").length;
    const inProgress = tasks.filter((t) => t.status === "in_progress").length;
    const overdue = tasks.filter(
      (t) => t.dueDate && t.dueDate < new Date() && t.status !== "done"
    ).length;
    const totalTimeSpent = tasks.reduce(
      (acc, task) =>
        acc + task.timeEntries.reduce((sum, entry) => sum + entry.duration, 0),
      0
    );
    const totalEstimated = tasks.reduce((acc, task) => acc + (task.estimatedTime || 0), 0);
    const completionRate = total > 0 ? (completed / total) * 100 : 0;

    return {
      total,
      completed,
      inProgress,
      overdue,
      totalTimeSpent,
      totalEstimated,
      completionRate,
    };
  }, [tasks]);

  return (
    <div className="flex flex-col h-full bg-[#0A0A0A]">
      {/* Header */}
      <div className="flex-shrink-0 px-8 py-6 border-b border-[#2A2A2A]">
        <div className="flex items-center justify-between mb-4">
          <div>
            <h1 className="text-2xl font-bold text-white mb-1">Tasks</h1>
            <p className="text-sm text-gray-400">
              {analytics.total} tasks • {analytics.completed} completed • {analytics.inProgress} in progress
            </p>
          </div>
          <div className="flex items-center gap-2">
            <Button
              variant="outline"
              size="sm"
              onClick={() => {/* Export functionality */}}
            >
              <Download className="w-4 h-4 mr-2" />
              Export
            </Button>
            <Button
              variant="outline"
              size="sm"
              onClick={() => {/* Import functionality */}}
            >
              <Upload className="w-4 h-4 mr-2" />
              Import
            </Button>
            <Button
              onClick={() => handleAddTask("todo")}
              className="bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] text-white hover:opacity-90"
            >
              <Plus className="w-4 h-4 mr-2" />
              New Task
            </Button>
          </div>
        </div>

        {/* Filters and Search */}
        <div className="flex items-center gap-3">
          <div className="relative flex-1">
            <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-gray-500" />
            <Input
              type="text"
              placeholder="Search tasks..."
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              className="pl-10"
            />
          </div>

          <Select value={filterPriority} onValueChange={(value) => setFilterPriority(value as TaskPriority | "all")}>
            <SelectTrigger className="w-[140px]">
              <SelectValue placeholder="Priority" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="all">All Priorities</SelectItem>
              <SelectItem value="urgent">Urgent</SelectItem>
              <SelectItem value="high">High</SelectItem>
              <SelectItem value="medium">Medium</SelectItem>
              <SelectItem value="low">Low</SelectItem>
            </SelectContent>
          </Select>

          <Select value={sortBy} onValueChange={(value) => setSortBy(value as any)}>
            <SelectTrigger className="w-[140px]">
              <SelectValue placeholder="Sort by" />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="dueDate">Due Date</SelectItem>
              <SelectItem value="priority">Priority</SelectItem>
              <SelectItem value="created">Created</SelectItem>
              <SelectItem value="updated">Updated</SelectItem>
            </SelectContent>
          </Select>

          <Tabs value={viewMode} onValueChange={(value) => setViewMode(value as ViewMode)}>
            <TabsList>
              <TabsTrigger value="kanban">Kanban</TabsTrigger>
              <TabsTrigger value="list">List</TabsTrigger>
              <TabsTrigger value="calendar">Calendar</TabsTrigger>
              <TabsTrigger value="analytics">
                <BarChart3 className="w-4 h-4" />
              </TabsTrigger>
            </TabsList>
          </Tabs>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 overflow-hidden">
        {viewMode === "kanban" && (
          <DndContext
            sensors={sensors}
            collisionDetection={closestCorners}
            onDragStart={handleDragStart}
            onDragOver={handleDragOver}
            onDragEnd={handleDragEnd}
          >
            <div className="h-full px-8 py-6 overflow-x-auto">
              <div className="flex gap-6 h-full">
                <div id="todo">
                  <KanbanColumn
                    status="todo"
                    tasks={tasksByStatus.todo}
                    onViewDetails={handleViewDetails}
                    onQuickEdit={handleQuickEdit}
                    onDelete={handleDelete}
                    onAddTask={handleAddTask}
                  />
                </div>
                <div id="in_progress">
                  <KanbanColumn
                    status="in_progress"
                    tasks={tasksByStatus.in_progress}
                    onViewDetails={handleViewDetails}
                    onQuickEdit={handleQuickEdit}
                    onDelete={handleDelete}
                    onAddTask={handleAddTask}
                  />
                </div>
                <div id="review">
                  <KanbanColumn
                    status="review"
                    tasks={tasksByStatus.review}
                    onViewDetails={handleViewDetails}
                    onQuickEdit={handleQuickEdit}
                    onDelete={handleDelete}
                    onAddTask={handleAddTask}
                  />
                </div>
                <div id="done">
                  <KanbanColumn
                    status="done"
                    tasks={tasksByStatus.done}
                    onViewDetails={handleViewDetails}
                    onQuickEdit={handleQuickEdit}
                    onDelete={handleDelete}
                    onAddTask={handleAddTask}
                  />
                </div>
              </div>
            </div>

            <DragOverlay>
              {activeId ? (
                <div className="opacity-50">
                  <SortableTaskCard
                    task={tasks.find((t) => t.id === activeId)!}
                    onViewDetails={() => {}}
                    onQuickEdit={() => {}}
                    onDelete={() => {}}
                  />
                </div>
              ) : null}
            </DragOverlay>
          </DndContext>
        )}

        {viewMode === "analytics" && (
          <div className="h-full px-8 py-6 overflow-y-auto">
            <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
              <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
                <div className="flex items-center justify-between mb-2">
                  <span className="text-sm text-gray-400">Total Tasks</span>
                  <TrendingUp className="w-4 h-4 text-[#00D4FF]" />
                </div>
                <div className="text-3xl font-bold text-white">{analytics.total}</div>
              </div>
              <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
                <div className="flex items-center justify-between mb-2">
                  <span className="text-sm text-gray-400">Completed</span>
                  <CheckCircle2 className="w-4 h-4 text-[#00E676]" />
                </div>
                <div className="text-3xl font-bold text-white">{analytics.completed}</div>
                <div className="text-xs text-gray-500 mt-1">
                  {analytics.completionRate.toFixed(1)}% completion rate
                </div>
              </div>
              <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
                <div className="flex items-center justify-between mb-2">
                  <span className="text-sm text-gray-400">In Progress</span>
                  <Timer className="w-4 h-4 text-[#00D4FF]" />
                </div>
                <div className="text-3xl font-bold text-white">{analytics.inProgress}</div>
              </div>
              <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
                <div className="flex items-center justify-between mb-2">
                  <span className="text-sm text-gray-400">Overdue</span>
                  <AlertCircle className="w-4 h-4 text-red-400" />
                </div>
                <div className="text-3xl font-bold text-white">{analytics.overdue}</div>
              </div>
            </div>

            <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
              <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
                <h3 className="text-lg font-semibold text-white mb-4">Time Tracking</h3>
                <div className="space-y-4">
                  <div>
                    <div className="flex items-center justify-between text-sm mb-2">
                      <span className="text-gray-400">Time Spent</span>
                      <span className="text-white font-medium">
                        {Math.floor(analytics.totalTimeSpent / 60)}h {analytics.totalTimeSpent % 60}m
                      </span>
                    </div>
                    <div>
                      <div className="flex items-center justify-between text-sm mb-2">
                        <span className="text-gray-400">Estimated Time</span>
                        <span className="text-white font-medium">
                          {Math.floor(analytics.totalEstimated / 60)}h {analytics.totalEstimated % 60}m
                        </span>
                      </div>
                    </div>
                    <Progress
                      value={Math.min((analytics.totalTimeSpent / analytics.totalEstimated) * 100, 100)}
                      className="h-2"
                    />
                  </div>
                </div>
              </div>

              <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
                <h3 className="text-lg font-semibold text-white mb-4">Priority Distribution</h3>
                <div className="space-y-3">
                  {(["urgent", "high", "medium", "low"] as TaskPriority[]).map((priority) => {
                    const count = tasks.filter((t) => t.priority === priority).length;
                    const percentage = tasks.length > 0 ? (count / tasks.length) * 100 : 0;
                    return (
                      <div key={priority}>
                        <div className="flex items-center justify-between text-sm mb-1">
                          <span className={priorityConfig[priority].color}>
                            {priorityConfig[priority].label}
                          </span>
                          <span className="text-white">{count}</span>
                        </div>
                        <Progress value={percentage} className="h-2" />
                      </div>
                    );
                  })}
                </div>
              </div>
            </div>
          </div>
        )}
      </div>

      {/* Task Detail Dialog */}
      <TaskDetailDialog
        task={selectedTask}
        open={isTaskDialogOpen}
        onOpenChange={setIsTaskDialogOpen}
        onEdit={(task) => {
          setIsTaskDialogOpen(false);
          setSelectedTask(task);
          setIsCreateDialogOpen(true);
        }}
      />

      {/* Task Create/Edit Dialog */}
      <TaskFormDialog
        task={selectedTask}
        open={isCreateDialogOpen}
        onOpenChange={setIsCreateDialogOpen}
        onSave={handleSaveTask}
      />
    </div>
  );
}
