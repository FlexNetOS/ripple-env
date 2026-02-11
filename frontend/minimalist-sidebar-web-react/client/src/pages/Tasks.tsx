import { useState } from "react";
import {
  DndContext,
  DragEndEvent,
  DragOverlay,
  DragStartEvent,
  PointerSensor,
  useSensor,
  useSensors,
} from "@dnd-kit/core";
import { SortableContext, verticalListSortingStrategy } from "@dnd-kit/sortable";
import { motion } from "framer-motion";
import { Plus, Search } from "lucide-react";

type TaskStatus = "todo" | "in_progress" | "review" | "done";

interface Task {
  id: string;
  title: string;
  description: string;
  status: TaskStatus;
  priority: "low" | "medium" | "high";
  assignee?: string;
  dueDate?: string;
  tags: string[];
}

const mockTasks: Task[] = [
  {
    id: "1",
    title: "Design new landing page",
    description: "Create wireframes and mockups for the new landing page",
    status: "in_progress",
    priority: "high",
    assignee: "Demo User",
    dueDate: "2026-02-15",
    tags: ["design", "ui/ux"],
  },
  {
    id: "2",
    title: "Implement authentication",
    description: "Add OAuth support for Google and GitHub",
    status: "todo",
    priority: "high",
    tags: ["backend", "security"],
  },
  {
    id: "3",
    title: "Write API documentation",
    description: "Document all REST API endpoints",
    status: "todo",
    priority: "medium",
    tags: ["docs"],
  },
  {
    id: "4",
    title: "Fix mobile responsiveness",
    description: "Ensure all pages work on mobile devices",
    status: "in_progress",
    priority: "medium",
    assignee: "Demo User",
    tags: ["frontend", "mobile"],
  },
  {
    id: "5",
    title: "Set up CI/CD pipeline",
    description: "Configure GitHub Actions for automated testing and deployment",
    status: "review",
    priority: "high",
    tags: ["devops"],
  },
  {
    id: "6",
    title: "Database optimization",
    description: "Add indexes and optimize slow queries",
    status: "done",
    priority: "low",
    tags: ["backend", "performance"],
  },
];

const statusConfig: Record<
  TaskStatus,
  { label: string; color: string; bgColor: string; count: number }
> = {
  todo: {
    label: "To Do",
    color: "text-gray-400",
    bgColor: "bg-gray-500/10",
    count: 0,
  },
  in_progress: {
    label: "In Progress",
    color: "text-[#00D4FF]",
    bgColor: "bg-[#00D4FF]/10",
    count: 0,
  },
  review: {
    label: "Review",
    color: "text-[#9B7BFF]",
    bgColor: "bg-[#9B7BFF]/10",
    count: 0,
  },
  done: {
    label: "Done",
    color: "text-[#00E676]",
    bgColor: "bg-[#00E676]/10",
    count: 0,
  },
};

const priorityColors = {
  low: "bg-gray-500/20 text-gray-400",
  medium: "bg-[#00D4FF]/20 text-[#00D4FF]",
  high: "bg-red-500/20 text-red-400",
};

function TaskCard({ task }: { task: Task }) {
  return (
    <motion.div
      layout
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      exit={{ opacity: 0, y: -20 }}
      className="group relative bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg p-4 hover:border-[#00D4FF]/30 transition-all cursor-pointer"
    >
      {/* Priority indicator */}
      <div className="absolute top-0 left-0 w-1 h-full rounded-l-lg bg-gradient-to-b from-[#00D4FF] to-[#9B7BFF] opacity-0 group-hover:opacity-100 transition-opacity" />

      <div className="space-y-3">
        {/* Header */}
        <div className="flex items-start justify-between gap-2">
          <h3 className="font-medium text-white text-sm line-clamp-2">{task.title}</h3>
          <span
            className={`px-2 py-0.5 rounded text-xs font-medium ${priorityColors[task.priority]}`}
          >
            {task.priority}
          </span>
        </div>

        {/* Description */}
        <p className="text-sm text-gray-400 line-clamp-2">{task.description}</p>

        {/* Tags */}
        {task.tags.length > 0 && (
          <div className="flex flex-wrap gap-1.5">
            {task.tags.map((tag) => (
              <span
                key={tag}
                className="px-2 py-0.5 rounded-full text-xs bg-[#2A2A2A] text-gray-300"
              >
                {tag}
              </span>
            ))}
          </div>
        )}

        {/* Footer */}
        <div className="flex items-center justify-between text-xs text-gray-500">
          {task.assignee && <span>{task.assignee}</span>}
          {task.dueDate && <span>{new Date(task.dueDate).toLocaleDateString()}</span>}
        </div>
      </div>
    </motion.div>
  );
}

function KanbanColumn({
  status,
  tasks,
}: {
  status: TaskStatus;
  tasks: Task[];
}) {
  const config = statusConfig[status];

  return (
    <div className="flex flex-col h-full min-w-[320px]">
      {/* Column Header */}
      <div className="flex items-center justify-between mb-4 pb-3 border-b border-[#2A2A2A]">
        <div className="flex items-center gap-2">
          <div className={`w-2 h-2 rounded-full ${config.bgColor}`} />
          <h2 className={`font-semibold ${config.color}`}>{config.label}</h2>
          <span className="px-2 py-0.5 rounded-full text-xs bg-[#2A2A2A] text-gray-400">
            {tasks.length}
          </span>
        </div>
        <button className="p-1.5 rounded hover:bg-[#2A2A2A] transition-colors">
          <Plus className="w-4 h-4 text-gray-400" />
        </button>
      </div>

      {/* Tasks List */}
      <SortableContext items={tasks.map((t) => t.id)} strategy={verticalListSortingStrategy}>
        <div className="flex-1 space-y-3 overflow-y-auto pr-2 scrollbar-thin scrollbar-thumb-[#2A2A2A] scrollbar-track-transparent">
          {tasks.map((task) => (
            <TaskCard key={task.id} task={task} />
          ))}
        </div>
      </SortableContext>
    </div>
  );
}

export default function Tasks() {
  const [tasks, setTasks] = useState<Task[]>(mockTasks);
  const [activeId, setActiveId] = useState<string | null>(null);
  const [searchQuery, setSearchQuery] = useState("");

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

  const handleDragEnd = (event: DragEndEvent) => {
    const { active, over } = event;

    if (!over) {
      setActiveId(null);
      return;
    }

    // Handle drag end logic here
    // You would update the task status based on which column it was dropped in

    setActiveId(null);
  };

  const filteredTasks = tasks.filter(
    (task) =>
      task.title.toLowerCase().includes(searchQuery.toLowerCase()) ||
      task.description.toLowerCase().includes(searchQuery.toLowerCase()) ||
      task.tags.some((tag) => tag.toLowerCase().includes(searchQuery.toLowerCase()))
  );

  const tasksByStatus = {
    todo: filteredTasks.filter((t) => t.status === "todo"),
    in_progress: filteredTasks.filter((t) => t.status === "in_progress"),
    review: filteredTasks.filter((t) => t.status === "review"),
    done: filteredTasks.filter((t) => t.status === "done"),
  };

  return (
    <div className="flex flex-col h-full bg-[#0A0A0A]">
      {/* Header */}
      <div className="flex-shrink-0 px-8 py-6 border-b border-[#2A2A2A]">
        <div className="flex items-center justify-between mb-4">
          <div>
            <h1 className="text-2xl font-bold text-white mb-1">Tasks</h1>
            <p className="text-sm text-gray-400">
              Manage your tasks and track progress
            </p>
          </div>
          <button className="px-4 py-2 bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] text-white rounded-lg hover:opacity-90 transition-opacity flex items-center gap-2">
            <Plus className="w-4 h-4" />
            New Task
          </button>
        </div>

        {/* Search */}
        <div className="relative">
          <Search className="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4 text-gray-500" />
          <input
            type="text"
            placeholder="Search tasks..."
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
            className="w-full pl-10 pr-4 py-2 bg-[#1A1A1A] border border-[#2A2A2A] rounded-lg text-white placeholder-gray-500 focus:outline-none focus:border-[#00D4FF]/50"
          />
        </div>
      </div>

      {/* Kanban Board */}
      <div className="flex-1 overflow-hidden">
        <DndContext
          sensors={sensors}
          onDragStart={handleDragStart}
          onDragEnd={handleDragEnd}
        >
          <div className="h-full px-8 py-6 overflow-x-auto">
            <div className="flex gap-6 h-full">
              <KanbanColumn status="todo" tasks={tasksByStatus.todo} />
              <KanbanColumn status="in_progress" tasks={tasksByStatus.in_progress} />
              <KanbanColumn status="review" tasks={tasksByStatus.review} />
              <KanbanColumn status="done" tasks={tasksByStatus.done} />
            </div>
          </div>

          <DragOverlay>
            {activeId ? (
              <TaskCard task={tasks.find((t) => t.id === activeId)!} />
            ) : null}
          </DragOverlay>
        </DndContext>
      </div>
    </div>
  );
}
