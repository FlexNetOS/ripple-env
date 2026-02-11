import { useState, useMemo, useCallback, useRef, useEffect } from "react";
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
  Loader2,
  XCircle,
  Link2,
  ChevronRight,
  ChevronDown,
  Users,
} from "lucide-react";
import { TaskDetailDialog, TaskFormDialog } from "@/components/tasks/TaskDialogs";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select";
import { Badge } from "@/components/ui/badge";
import { DropdownMenu, DropdownMenuContent, DropdownMenuItem, DropdownMenuSeparator, DropdownMenuTrigger } from "@/components/ui/dropdown-menu";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Progress } from "@/components/ui/progress";
import { Avatar, AvatarFallback, AvatarImage } from "@/components/ui/avatar";
import { Separator } from "@/components/ui/separator";
import { ScrollArea } from "@/components/ui/scroll-area";

// Enhanced types with vibe-kanban features
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
  duration: number;
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
  estimatedTime?: number;
  progress: number;
  dependencies: string[];
  starred: boolean;
  createdAt: Date;
  updatedAt: Date;
  // Vibe-kanban inspired features
  parent_task_id?: string;
  subtasks?: Task[];
  shared_task_id?: string;
  project_id?: string;
  has_in_progress_attempt?: boolean;
  last_attempt_failed?: boolean;
  executor?: string;
}

// Enhanced mock data with vibe-kanban features
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
      {
        id: "t2",
        duration: 60,
        description: "Mockup refinements",
        timestamp: new Date("2026-02-11T14:00:00"),
      },
    ],
    estimatedTime: 480,
    progress: 40,
    dependencies: [],
    starred: true,
    createdAt: new Date("2026-02-09T09:00:00"),
    updatedAt: new Date("2026-02-10T10:00:00"),
    has_in_progress_attempt: true,
    project_id: "proj-1",
  },
  {
    id: "1-1",
    title: "Create hero section mockup",
    description: "Design the hero section with call-to-action button",
    status: "done",
    priority: "medium",
    assignee: "Demo User",
    dueDate: new Date("2026-02-13"),
    tags: ["design", "subtask"],
    comments: [],
    activities: [
      {
        id: "a1-1",
        type: "created",
        user: "Demo User",
        description: "created this subtask",
        timestamp: new Date("2026-02-09T10:00:00"),
      },
    ],
    timeEntries: [
      {
        id: "t1-1",
        duration: 90,
        description: "Hero section design",
        timestamp: new Date("2026-02-10T11:00:00"),
      },
    ],
    estimatedTime: 120,
    progress: 100,
    dependencies: [],
    starred: false,
    createdAt: new Date("2026-02-09T10:00:00"),
    updatedAt: new Date("2026-02-10T16:00:00"),
    parent_task_id: "1",
    project_id: "proj-1",
  },
  {
    id: "1-2",
    title: "Design features section",
    description: "Create layout for product features showcase",
    status: "in_progress",
    priority: "medium",
    assignee: "Demo User",
    dueDate: new Date("2026-02-14"),
    tags: ["design", "subtask"],
    comments: [],
    activities: [
      {
        id: "a1-2",
        type: "created",
        user: "Demo User",
        description: "created this subtask",
        timestamp: new Date("2026-02-09T10:30:00"),
      },
    ],
    timeEntries: [
      {
        id: "t1-2",
        duration: 45,
        description: "Initial layout sketches",
        timestamp: new Date("2026-02-11T09:00:00"),
      },
    ],
    estimatedTime: 180,
    progress: 30,
    dependencies: [],
    starred: false,
    createdAt: new Date("2026-02-09T10:30:00"),
    updatedAt: new Date("2026-02-11T09:45:00"),
    parent_task_id: "1",
    project_id: "proj-1",
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
    project_id: "proj-1",
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
    project_id: "proj-1",
  },
  {
    id: "4",
    title: "Fix mobile responsiveness",
    description: "Ensure all pages work seamlessly on mobile devices and tablets",
    status: "review",
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
        description: "moved from In Progress to Review",
        timestamp: new Date("2026-02-11T14:00:00"),
      },
    ],
    timeEntries: [
      {
        id: "t3",
        duration: 90,
        description: "Mobile testing",
        timestamp: new Date("2026-02-11T15:00:00"),
      },
    ],
    estimatedTime: 180,
    progress: 85,
    dependencies: [],
    starred: false,
    createdAt: new Date("2026-02-06T10:00:00"),
    updatedAt: new Date("2026-02-11T15:00:00"),
    project_id: "proj-1",
  },
  {
    id: "5",
    title: "Setup CI/CD pipeline",
    description: "Configure automated testing and deployment with GitHub Actions",
    status: "done",
    priority: "medium",
    assignee: "Admin",
    dueDate: new Date("2026-01-25"),
    tags: ["devops", "ci/cd", "automation"],
    comments: [
      {
        id: "c3",
        author: "Admin",
        content: "Pipeline is working great!",
        timestamp: new Date("2026-01-26T10:00:00"),
      },
    ],
    activities: [
      {
        id: "a7",
        type: "created",
        user: "Admin",
        description: "created this task",
        timestamp: new Date("2026-01-20T09:00:00"),
      },
      {
        id: "a8",
        type: "status_changed",
        user: "Admin",
        description: "moved from In Progress to Done",
        timestamp: new Date("2026-01-25T17:00:00"),
      },
    ],
    timeEntries: [
      {
        id: "t4",
        duration: 240,
        description: "CI/CD setup and configuration",
        timestamp: new Date("2026-01-24T10:00:00"),
      },
    ],
    estimatedTime: 360,
    progress: 100,
    dependencies: [],
    starred: true,
    createdAt: new Date("2026-01-20T09:00:00"),
    updatedAt: new Date("2026-01-25T17:00:00"),
    project_id: "proj-1",
  },
  {
    id: "6",
    title: "Refactor legacy code",
    description: "Clean up old codebase and improve performance",
    status: "cancelled",
    priority: "low",
    assignee: undefined,
    dueDate: new Date("2026-03-01"),
    tags: ["refactoring", "technical-debt"],
    comments: [
      {
        id: "c4",
        author: "Admin",
        content: "Decided to rewrite instead of refactor",
        timestamp: new Date("2026-02-09T11:00:00"),
      },
    ],
    activities: [
      {
        id: "a9",
        type: "created",
        user: "Admin",
        description: "created this task",
        timestamp: new Date("2026-02-05T14:00:00"),
      },
      {
        id: "a10",
        type: "status_changed",
        user: "Admin",
        description: "cancelled this task",
        timestamp: new Date("2026-02-09T11:00:00"),
      },
    ],
    timeEntries: [
      {
        id: "t5",
        duration: 30,
        description: "Initial code review",
        timestamp: new Date("2026-02-06T10:00:00"),
      },
    ],
    estimatedTime: 480,
    progress: 5,
    dependencies: [],
    starred: false,
    createdAt: new Date("2026-02-05T14:00:00"),
    updatedAt: new Date("2026-02-09T11:00:00"),
    project_id: "proj-1",
  },
  {
    id: "7",
    title: "Shared Task: Database Migration",
    description: "Migrate from PostgreSQL to MySQL for better performance",
    status: "in_progress",
    priority: "urgent",
    assignee: "Team Lead",
    dueDate: new Date("2026-02-16"),
    tags: ["database", "migration", "shared"],
    comments: [],
    activities: [
      {
        id: "a11",
        type: "created",
        user: "Team Lead",
        description: "created this shared task",
        timestamp: new Date("2026-02-08T09:00:00"),
      },
    ],
    timeEntries: [],
    estimatedTime: 600,
    progress: 20,
    dependencies: [],
    starred: false,
    createdAt: new Date("2026-02-08T09:00:00"),
    updatedAt: new Date("2026-02-10T14:00:00"),
    shared_task_id: "shared-1",
    project_id: "proj-2",
  },
  {
    id: "8",
    title: "Failed Task: Load Testing",
    description: "Perform load testing on production environment",
    status: "todo",
    priority: "high",
    assignee: "Demo User",
    dueDate: new Date("2026-02-18"),
    tags: ["testing", "performance"],
    comments: [],
    activities: [
      {
        id: "a12",
        type: "created",
        user: "Demo User",
        description: "created this task",
        timestamp: new Date("2026-02-10T08:00:00"),
      },
    ],
    timeEntries: [],
    estimatedTime: 240,
    progress: 0,
    dependencies: [],
    starred: false,
    createdAt: new Date("2026-02-10T08:00:00"),
    updatedAt: new Date("2026-02-10T08:00:00"),
    last_attempt_failed: true,
    project_id: "proj-1",
  },
];

const statusConfig = {
  todo: {
    label: "To Do",
    color: "bg-gray-500",
    icon: Clock,
  },
  in_progress: {
    label: "In Progress",
    color: "bg-cyan-500",
    icon: Zap,
  },
  review: {
    label: "Review",
    color: "bg-purple-500",
    icon: AlertCircle,
  },
  done: {
    label: "Done",
    color: "bg-green-500",
    icon: CheckCircle2,
  },
  cancelled: {
    label: "Cancelled",
    color: "bg-red-500",
    icon: XCircle,
  },
};

const priorityConfig = {
  low: { label: "Low", color: "text-gray-400 border-gray-400" },
  medium: { label: "Medium", color: "text-cyan-400 border-cyan-400" },
  high: { label: "High", color: "text-orange-400 border-orange-400" },
  urgent: { label: "Urgent", color: "text-red-400 border-red-400" },
};

export default function TasksEnhancedV2() {
  const [tasks, setTasks] = useState<Task[]>(mockTasks);
  const [searchQuery, setSearchQuery] = useState("");
  const [priorityFilter, setPriorityFilter] = useState<string>("all");
  const [sortBy, setSortBy] = useState<string>("dueDate");
  const [viewMode, setViewMode] = useState<ViewMode>("kanban");
  const [selectedTask, setSelectedTask] = useState<Task | null>(null);
  const [isTaskDialogOpen, setIsTaskDialogOpen] = useState(false);
  const [isCreateDialogOpen, setIsCreateDialogOpen] = useState(false);
  const [activeId, setActiveId] = useState<string | null>(null);
  const [expandedTasks, setExpandedTasks] = useState<Set<string>>(new Set());
  const [columnToAddTask, setColumnToAddTask] = useState<TaskStatus | null>(null);

  const sensors = useSensors(
    useSensor(PointerSensor, {
      activationConstraint: {
        distance: 8,
      },
    })
  );

  // Filter and sort tasks
  const filteredAndSortedTasks = useMemo(() => {
    let filtered = tasks.filter((task) => {
      const matchesSearch =
        task.title.toLowerCase().includes(searchQuery.toLowerCase()) ||
        task.description.toLowerCase().includes(searchQuery.toLowerCase()) ||
        task.tags.some((tag) => tag.toLowerCase().includes(searchQuery.toLowerCase()));

      const matchesPriority = priorityFilter === "all" || task.priority === priorityFilter;

      return matchesSearch && matchesPriority;
    });

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
  }, [tasks, searchQuery, priorityFilter, sortBy]);

  // Group tasks by status (excluding subtasks from top level)
  const tasksByStatus = useMemo(() => {
    const topLevelTasks = filteredAndSortedTasks.filter(task => !task.parent_task_id);
    
    return {
      todo: topLevelTasks.filter((t) => t.status === "todo"),
      in_progress: topLevelTasks.filter((t) => t.status === "in_progress"),
      review: topLevelTasks.filter((t) => t.status === "review"),
      done: topLevelTasks.filter((t) => t.status === "done"),
      cancelled: topLevelTasks.filter((t) => t.status === "cancelled"),
    };
  }, [filteredAndSortedTasks]);

  // Get subtasks for a parent task
  const getSubtasks = useCallback((parentId: string): Task[] => {
    return tasks.filter(task => task.parent_task_id === parentId);
  }, [tasks]);

  const handleDragStart = (event: DragStartEvent) => {
    setActiveId(event.active.id as string);
  };

  const handleDragEnd = (event: DragEndEvent) => {
    const { active, over } = event;

    if (!over) {
      setActiveId(null);
      return;
    }

    const activeId = active.id as string;
    const overId = over.id as string;

    const activeTask = tasks.find((t) => t.id === activeId);
    if (!activeTask) {
      setActiveId(null);
      return;
    }

    // Don't allow dragging shared tasks
    if (activeTask.shared_task_id) {
      setActiveId(null);
      return;
    }

    // Check if dropped on a status column
    const newStatus = overId as TaskStatus;
    if (["todo", "in_progress", "review", "done", "cancelled"].includes(newStatus)) {
      if (activeTask.status !== newStatus) {
        setTasks((prevTasks) =>
          prevTasks.map((task) =>
            task.id === activeId
              ? {
                  ...task,
                  status: newStatus,
                  updatedAt: new Date(),
                  activities: [
                    ...task.activities,
                    {
                      id: `activity-${Date.now()}`,
                      type: "status_changed" as const,
                      user: "Demo User",
                      description: `moved from ${statusConfig[task.status].label} to ${statusConfig[newStatus].label}`,
                      timestamp: new Date(),
                    },
                  ],
                }
              : task
          )
        );
      }
    }

    setActiveId(null);
  };

  const handleTaskClick = (task: Task) => {
    setSelectedTask(task);
    setIsTaskDialogOpen(true);
  };

  const handleCreateTask = (status?: TaskStatus) => {
    if (status) {
      setColumnToAddTask(status);
    }
    setSelectedTask(null);
    setIsCreateDialogOpen(true);
  };

  const handleEditTask = (task: Task) => {
    setSelectedTask(task);
    setIsTaskDialogOpen(false);
    setIsCreateDialogOpen(true);
  };

  const handleSaveTask = (task: Task) => {
    if (task.id.startsWith("task-")) {
      // New task
      const newTask = {
        ...task,
        id: `task-${Date.now()}`,
        status: columnToAddTask || task.status,
        createdAt: new Date(),
        updatedAt: new Date(),
        activities: [
          {
            id: `activity-${Date.now()}`,
            type: "created" as const,
            user: "Demo User",
            description: "created this task",
            timestamp: new Date(),
          },
        ],
      };
      setTasks([...tasks, newTask]);
      setColumnToAddTask(null);
    } else {
      // Update existing task
      setTasks(
        tasks.map((t) =>
          t.id === task.id
            ? {
                ...task,
                updatedAt: new Date(),
                activities: [
                  ...task.activities,
                  {
                    id: `activity-${Date.now()}`,
                    type: "updated" as const,
                    user: "Demo User",
                    description: "updated this task",
                    timestamp: new Date(),
                  },
                ],
              }
            : t
        )
      );
    }
    setIsCreateDialogOpen(false);
  };

  const handleDeleteTask = (taskId: string) => {
    setTasks(tasks.filter((t) => t.id !== taskId));
    setIsTaskDialogOpen(false);
  };

  const handleDuplicateTask = (task: Task) => {
    const newTask = {
      ...task,
      id: `task-${Date.now()}`,
      title: `${task.title} (Copy)`,
      createdAt: new Date(),
      updatedAt: new Date(),
      activities: [
        {
          id: `activity-${Date.now()}`,
          type: "created" as const,
          user: "Demo User",
          description: "created this task",
          timestamp: new Date(),
        },
      ],
    };
    setTasks([...tasks, newTask]);
  };

  const toggleTaskExpansion = (taskId: string) => {
    setExpandedTasks(prev => {
      const newSet = new Set(prev);
      if (newSet.has(taskId)) {
        newSet.delete(taskId);
      } else {
        newSet.add(taskId);
      }
      return newSet;
    });
  };

  // Calculate analytics
  const analytics = useMemo(() => {
    const total = tasks.length;
    const completed = tasks.filter((t) => t.status === "done").length;
    const inProgress = tasks.filter((t) => t.status === "in_progress").length;
    const overdue = tasks.filter(
      (t) => t.dueDate && t.dueDate < new Date() && t.status !== "done"
    ).length;

    const totalEstimated = tasks.reduce((sum, t) => sum + (t.estimatedTime || 0), 0);
    const totalSpent = tasks.reduce(
      (sum, t) => sum + t.timeEntries.reduce((s, e) => s + e.duration, 0),
      0
    );

    const priorityBreakdown = {
      urgent: tasks.filter((t) => t.priority === "urgent").length,
      high: tasks.filter((t) => t.priority === "high").length,
      medium: tasks.filter((t) => t.priority === "medium").length,
      low: tasks.filter((t) => t.priority === "low").length,
    };

    return {
      total,
      completed,
      inProgress,
      overdue,
      completionRate: total > 0 ? Math.round((completed / total) * 100) : 0,
      totalEstimated,
      totalSpent,
      priorityBreakdown,
    };
  }, [tasks]);

  return (
    <div className="flex flex-col h-full bg-[#0A0A0A] text-white">
      {/* Header */}
      <div className="flex-shrink-0 border-b border-[#2A2A2A] p-6">
        <div className="flex items-center justify-between mb-6">
          <div>
            <h1 className="text-3xl font-bold">Tasks</h1>
            <p className="text-sm text-gray-400 mt-1">
              {analytics.total} tasks • {analytics.completed} completed • {analytics.inProgress} in progress
            </p>
          </div>
          <div className="flex items-center gap-3">
            <Button
              variant="outline"
              size="sm"
              className="border-[#2A2A2A] hover:bg-[#1A1A1A]"
            >
              <Download className="h-4 w-4 mr-2" />
              Export
            </Button>
            <Button
              variant="outline"
              size="sm"
              className="border-[#2A2A2A] hover:bg-[#1A1A1A]"
            >
              <Upload className="h-4 w-4 mr-2" />
              Import
            </Button>
            <Button
              onClick={() => handleCreateTask()}
              className="bg-[#00D4FF] hover:bg-[#00B8E6] text-black"
            >
              <Plus className="h-4 w-4 mr-2" />
              New Task
            </Button>
          </div>
        </div>

        {/* Filters */}
        <div className="flex items-center gap-4 flex-wrap">
          <div className="flex-1 min-w-[300px]">
            <div className="relative">
              <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 h-4 w-4 text-gray-400" />
              <Input
                placeholder="Search tasks..."
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
                className="pl-10 bg-[#1A1A1A] border-[#2A2A2A] focus:border-[#00D4FF]"
              />
            </div>
          </div>

          <Select value={priorityFilter} onValueChange={setPriorityFilter}>
            <SelectTrigger className="w-[150px] bg-[#1A1A1A] border-[#2A2A2A]">
              <Filter className="h-4 w-4 mr-2" />
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="all">All Priorities</SelectItem>
              <SelectItem value="urgent">Urgent</SelectItem>
              <SelectItem value="high">High</SelectItem>
              <SelectItem value="medium">Medium</SelectItem>
              <SelectItem value="low">Low</SelectItem>
            </SelectContent>
          </Select>

          <Select value={sortBy} onValueChange={setSortBy}>
            <SelectTrigger className="w-[150px] bg-[#1A1A1A] border-[#2A2A2A]">
              <SortAsc className="h-4 w-4 mr-2" />
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              <SelectItem value="dueDate">Due Date</SelectItem>
              <SelectItem value="priority">Priority</SelectItem>
              <SelectItem value="created">Created</SelectItem>
              <SelectItem value="updated">Updated</SelectItem>
            </SelectContent>
          </Select>
        </div>
      </div>

      {/* View Mode Tabs */}
      <Tabs value={viewMode} onValueChange={(v) => setViewMode(v as ViewMode)} className="flex-1 flex flex-col">
        <TabsList className="flex-shrink-0 bg-[#1A1A1A] border-b border-[#2A2A2A] rounded-none px-6">
          <TabsTrigger value="kanban">Kanban</TabsTrigger>
          <TabsTrigger value="list">List</TabsTrigger>
          <TabsTrigger value="calendar">Calendar</TabsTrigger>
          <TabsTrigger value="analytics">
            <BarChart3 className="h-4 w-4 mr-2" />
            Analytics
          </TabsTrigger>
        </TabsList>

        <TabsContent value="kanban" className="flex-1 mt-0 p-6 overflow-hidden">
          <DndContext
            sensors={sensors}
            collisionDetection={closestCorners}
            onDragStart={handleDragStart}
            onDragEnd={handleDragEnd}
          >
            <div className="flex gap-6 h-full overflow-x-auto pb-4">
              {(Object.keys(tasksByStatus) as TaskStatus[]).map((status) => (
                <KanbanColumn
                  key={status}
                  status={status}
                  tasks={tasksByStatus[status]}
                  getSubtasks={getSubtasks}
                  expandedTasks={expandedTasks}
                  onTaskClick={handleTaskClick}
                  onToggleExpansion={toggleTaskExpansion}
                  onAddTask={() => handleCreateTask(status)}
                  onEditTask={handleEditTask}
                  onDeleteTask={handleDeleteTask}
                  onDuplicateTask={handleDuplicateTask}
                />
              ))}
            </div>
          </DndContext>
        </TabsContent>

        <TabsContent value="list" className="flex-1 mt-0 p-6">
          <div className="text-center text-gray-400 py-12">
            List view coming soon...
          </div>
        </TabsContent>

        <TabsContent value="calendar" className="flex-1 mt-0 p-6">
          <div className="text-center text-gray-400 py-12">
            Calendar view coming soon...
          </div>
        </TabsContent>

        <TabsContent value="analytics" className="flex-1 mt-0 p-6 overflow-auto">
          <AnalyticsDashboard analytics={analytics} />
        </TabsContent>
      </Tabs>

      {/* Dialogs */}
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

      <TaskFormDialog
        task={selectedTask}
        open={isCreateDialogOpen}
        onOpenChange={setIsCreateDialogOpen}
        onSave={handleSaveTask}
      />
    </div>
  );
}

// Kanban Column Component with Add Task Button
interface KanbanColumnProps {
  status: TaskStatus;
  tasks: Task[];
  getSubtasks: (parentId: string) => Task[];
  expandedTasks: Set<string>;
  onTaskClick: (task: Task) => void;
  onToggleExpansion: (taskId: string) => void;
  onAddTask: () => void;
  onEditTask: (task: Task) => void;
  onDeleteTask: (taskId: string) => void;
  onDuplicateTask: (task: Task) => void;
}

function KanbanColumn({
  status,
  tasks,
  getSubtasks,
  expandedTasks,
  onTaskClick,
  onToggleExpansion,
  onAddTask,
  onEditTask,
  onDeleteTask,
  onDuplicateTask,
}: KanbanColumnProps) {
  const config = statusConfig[status];
  const StatusIcon = config.icon;

  return (
    <div className="flex-shrink-0 w-[350px] flex flex-col bg-[#1A1A1A] rounded-lg border border-[#2A2A2A]">
      {/* Column Header with Add Button */}
      <div className="flex items-center justify-between p-4 border-b border-[#2A2A2A]">
        <div className="flex items-center gap-2">
          <div className={`w-3 h-3 rounded-full ${config.color}`} />
          <h3 className="font-semibold">{config.label}</h3>
          <Badge variant="secondary" className="ml-2">
            {tasks.length}
          </Badge>
        </div>
        <Button
          variant="ghost"
          size="sm"
          onClick={onAddTask}
          className="h-8 w-8 p-0 hover:bg-[#2A2A2A]"
          title="Add task"
        >
          <Plus className="h-4 w-4" />
        </Button>
      </div>

      {/* Tasks */}
      <ScrollArea className="flex-1 p-4">
        <SortableContext items={tasks.map((t) => t.id)} strategy={verticalListSortingStrategy}>
          <div className="space-y-3">
            {tasks.map((task) => (
              <TaskCardWithSubtasks
                key={task.id}
                task={task}
                subtasks={getSubtasks(task.id)}
                isExpanded={expandedTasks.has(task.id)}
                onTaskClick={onTaskClick}
                onToggleExpansion={onToggleExpansion}
                onEdit={onEditTask}
                onDelete={onDeleteTask}
                onDuplicate={onDuplicateTask}
              />
            ))}
            {tasks.length === 0 && (
              <div className="text-center text-gray-500 py-8">
                <StatusIcon className="h-8 w-8 mx-auto mb-2 opacity-50" />
                <p className="text-sm">No tasks</p>
              </div>
            )}
          </div>
        </SortableContext>
      </ScrollArea>
    </div>
  );
}

// Enhanced Task Card with Subtasks, Visual Indicators, and Scroll Support
interface TaskCardWithSubtasksProps {
  task: Task;
  subtasks: Task[];
  isExpanded: boolean;
  onTaskClick: (task: Task) => void;
  onToggleExpansion: (taskId: string) => void;
  onEdit: (task: Task) => void;
  onDelete: (taskId: string) => void;
  onDuplicate: (task: Task) => void;
}

function TaskCardWithSubtasks({
  task,
  subtasks,
  isExpanded,
  onTaskClick,
  onToggleExpansion,
  onEdit,
  onDelete,
  onDuplicate,
}: TaskCardWithSubtasksProps) {
  const {
    attributes,
    listeners,
    setNodeRef,
    transform,
    transition,
    isDragging,
  } = useSortable({
    id: task.id,
    disabled: !!task.shared_task_id, // Disable drag for shared tasks
  });

  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
    opacity: isDragging ? 0.5 : 1,
  };

  const cardRef = useRef<HTMLDivElement>(null);

  // Scroll into view when selected
  useEffect(() => {
    if (task.starred && cardRef.current) {
      cardRef.current.scrollIntoView({
        behavior: "smooth",
        block: "center",
      });
    }
  }, [task.starred]);

  const hasSubtasks = subtasks.length > 0;
  const completedSubtasks = subtasks.filter(st => st.status === "done").length;

  return (
    <div ref={setNodeRef} style={style} {...attributes} {...listeners}>
      <motion.div
        ref={cardRef}
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        exit={{ opacity: 0, y: -20 }}
        className={`
          bg-[#0A0A0A] border border-[#2A2A2A] rounded-lg p-4 cursor-pointer
          hover:border-[#00D4FF] transition-all
          ${task.shared_task_id ? "border-l-4 border-l-cyan-500" : ""}
          ${task.starred ? "ring-2 ring-yellow-500" : ""}
        `}
        onClick={() => onTaskClick(task)}
      >
        <div className="flex flex-col gap-3">
          {/* Header with Title and Indicators */}
          <div className="flex items-start justify-between gap-2">
            <div className="flex-1 min-w-0">
              <div className="flex items-center gap-2 mb-1">
                {hasSubtasks && (
                  <button
                    onClick={(e) => {
                      e.stopPropagation();
                      onToggleExpansion(task.id);
                    }}
                    className="flex-shrink-0"
                  >
                    {isExpanded ? (
                      <ChevronDown className="h-4 w-4 text-gray-400" />
                    ) : (
                      <ChevronRight className="h-4 w-4 text-gray-400" />
                    )}
                  </button>
                )}
                <h4 className="font-medium truncate">{task.title}</h4>
              </div>
              {task.description && (
                <p className="text-sm text-gray-400 line-clamp-2">
                  {task.description}
                </p>
              )}
            </div>

            {/* Visual Indicators */}
            <div className="flex items-center gap-1 flex-shrink-0">
              {task.has_in_progress_attempt && (
                <Loader2 className="h-4 w-4 animate-spin text-cyan-500" title="In progress" />
              )}
              {task.last_attempt_failed && (
                <XCircle className="h-4 w-4 text-red-500" title="Last attempt failed" />
              )}
              {task.parent_task_id && (
                <Link2 className="h-4 w-4 text-purple-500" title="Has parent task" />
              )}
              {task.shared_task_id && (
                <Users className="h-4 w-4 text-cyan-500" title="Shared task" />
              )}
              {task.starred && (
                <Star className="h-4 w-4 text-yellow-500 fill-yellow-500" title="Starred" />
              )}
              
              {/* Actions Dropdown */}
              <DropdownMenu>
                <DropdownMenuTrigger asChild onClick={(e) => e.stopPropagation()}>
                  <Button
                    variant="ghost"
                    size="sm"
                    className="h-6 w-6 p-0 hover:bg-[#2A2A2A]"
                  >
                    <MoreVertical className="h-4 w-4" />
                  </Button>
                </DropdownMenuTrigger>
                <DropdownMenuContent align="end" className="bg-[#1A1A1A] border-[#2A2A2A]">
                  <DropdownMenuItem onClick={(e) => { e.stopPropagation(); onEdit(task); }}>
                    <Edit className="h-4 w-4 mr-2" />
                    Edit
                  </DropdownMenuItem>
                  <DropdownMenuItem onClick={(e) => { e.stopPropagation(); onDuplicate(task); }}>
                    <Copy className="h-4 w-4 mr-2" />
                    Duplicate
                  </DropdownMenuItem>
                  <DropdownMenuSeparator className="bg-[#2A2A2A]" />
                  <DropdownMenuItem
                    onClick={(e) => { e.stopPropagation(); onDelete(task.id); }}
                    className="text-red-500"
                  >
                    <Trash2 className="h-4 w-4 mr-2" />
                    Delete
                  </DropdownMenuItem>
                </DropdownMenuContent>
              </DropdownMenu>
            </div>
          </div>

          {/* Subtasks Progress */}
          {hasSubtasks && (
            <div className="flex items-center gap-2 text-xs text-gray-400">
              <CheckCircle2 className="h-3 w-3" />
              <span>{completedSubtasks}/{subtasks.length} subtasks</span>
              <Progress value={(completedSubtasks / subtasks.length) * 100} className="h-1 flex-1" />
            </div>
          )}

          {/* Tags */}
          {task.tags.length > 0 && (
            <div className="flex flex-wrap gap-1">
              {task.tags.slice(0, 3).map((tag) => (
                <Badge
                  key={tag}
                  variant="secondary"
                  className="text-xs bg-[#2A2A2A] text-gray-300"
                >
                  {tag}
                </Badge>
              ))}
              {task.tags.length > 3 && (
                <Badge variant="secondary" className="text-xs bg-[#2A2A2A] text-gray-300">
                  +{task.tags.length - 3}
                </Badge>
              )}
            </div>
          )}

          {/* Footer */}
          <div className="flex items-center justify-between text-xs text-gray-400">
            <div className="flex items-center gap-3">
              {task.priority && (
                <Badge
                  variant="outline"
                  className={priorityConfig[task.priority].color}
                >
                  {priorityConfig[task.priority].label}
                </Badge>
              )}
              {task.assignee && (
                <div className="flex items-center gap-1">
                  <User className="h-3 w-3" />
                  <span>{task.assignee}</span>
                </div>
              )}
            </div>
            {task.dueDate && (
              <div className="flex items-center gap-1">
                <Calendar className="h-3 w-3" />
                <span>{task.dueDate.toLocaleDateString()}</span>
              </div>
            )}
          </div>

          {/* Progress Bar */}
          {task.progress > 0 && (
            <div className="space-y-1">
              <div className="flex items-center justify-between text-xs text-gray-400">
                <span>Progress</span>
                <span>{task.progress}%</span>
              </div>
              <Progress value={task.progress} className="h-1" />
            </div>
          )}
        </div>

        {/* Expanded Subtasks */}
        {isExpanded && hasSubtasks && (
          <div className="mt-3 pl-4 border-l-2 border-[#2A2A2A] space-y-2">
            {subtasks.map((subtask) => (
              <div
                key={subtask.id}
                className="bg-[#1A1A1A] border border-[#2A2A2A] rounded p-2 text-sm cursor-pointer hover:border-[#00D4FF]"
                onClick={(e) => {
                  e.stopPropagation();
                  onTaskClick(subtask);
                }}
              >
                <div className="flex items-center justify-between">
                  <span className={subtask.status === "done" ? "line-through text-gray-500" : ""}>
                    {subtask.title}
                  </span>
                  {subtask.status === "done" && (
                    <CheckCircle2 className="h-3 w-3 text-green-500" />
                  )}
                </div>
              </div>
            ))}
          </div>
        )}
      </motion.div>
    </div>
  );
}

// Analytics Dashboard Component
interface AnalyticsDashboardProps {
  analytics: {
    total: number;
    completed: number;
    inProgress: number;
    overdue: number;
    completionRate: number;
    totalEstimated: number;
    totalSpent: number;
    priorityBreakdown: {
      urgent: number;
      high: number;
      medium: number;
      low: number;
    };
  };
}

function AnalyticsDashboard({ analytics }: AnalyticsDashboardProps) {
  return (
    <div className="space-y-6">
      {/* Metrics Cards */}
      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-4">
        <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
          <div className="flex items-center justify-between mb-2">
            <h3 className="text-sm font-medium text-gray-400">Total Tasks</h3>
            <BarChart3 className="h-5 w-5 text-cyan-500" />
          </div>
          <p className="text-3xl font-bold">{analytics.total}</p>
        </div>

        <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
          <div className="flex items-center justify-between mb-2">
            <h3 className="text-sm font-medium text-gray-400">Completed</h3>
            <CheckCircle2 className="h-5 w-5 text-green-500" />
          </div>
          <p className="text-3xl font-bold">{analytics.completed}</p>
          <p className="text-sm text-gray-400 mt-1">{analytics.completionRate}% completion rate</p>
        </div>

        <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
          <div className="flex items-center justify-between mb-2">
            <h3 className="text-sm font-medium text-gray-400">In Progress</h3>
            <Zap className="h-5 w-5 text-cyan-500" />
          </div>
          <p className="text-3xl font-bold">{analytics.inProgress}</p>
        </div>

        <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
          <div className="flex items-center justify-between mb-2">
            <h3 className="text-sm font-medium text-gray-400">Overdue</h3>
            <AlertCircle className="h-5 w-5 text-red-500" />
          </div>
          <p className="text-3xl font-bold">{analytics.overdue}</p>
        </div>
      </div>

      {/* Time Tracking */}
      <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Time Tracking</h3>
        <div className="space-y-4">
          <div>
            <div className="flex items-center justify-between mb-2">
              <span className="text-sm text-gray-400">Estimated Time</span>
              <span className="text-sm font-medium">{Math.round(analytics.totalEstimated / 60)}h</span>
            </div>
            <div className="flex items-center justify-between mb-2">
              <span className="text-sm text-gray-400">Time Spent</span>
              <span className="text-sm font-medium">{Math.round(analytics.totalSpent / 60)}h</span>
            </div>
            <Progress
              value={(analytics.totalSpent / analytics.totalEstimated) * 100}
              className="h-2"
            />
            <p className="text-xs text-gray-400 mt-2">
              {Math.round((analytics.totalSpent / analytics.totalEstimated) * 100)}% of estimated time used
            </p>
          </div>
        </div>
      </div>

      {/* Priority Distribution */}
      <div className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Priority Distribution</h3>
        <div className="space-y-3">
          {Object.entries(analytics.priorityBreakdown).map(([priority, count]) => (
            <div key={priority}>
              <div className="flex items-center justify-between mb-1">
                <span className="text-sm capitalize">{priority}</span>
                <span className="text-sm font-medium">{count}</span>
              </div>
              <Progress
                value={(count / analytics.total) * 100}
                className="h-2"
              />
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
