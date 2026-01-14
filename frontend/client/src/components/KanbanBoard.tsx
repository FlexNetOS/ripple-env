/**
 * Kanban Board Component
 * Integrates task management with drag-and-drop functionality
 * Compatible with vibe-kanban data structures
 */

import React, { useState, useCallback } from 'react';
import * as LucideIcons from 'lucide-react';
import { cn } from '@/lib/utils';
import { motion, AnimatePresence, Reorder } from 'framer-motion';

// Ripple brand colors
const COLORS = {
  primary: '#00D4FF',
  secondary: '#9B7BFF',
  accent: '#00E676',
  warning: '#FFB300',
  error: '#FF5252',
  foreground: '#ECEDEE',
  muted: '#9BA1A6',
  border: '#2A2A2A',
  surface: '#1A1A1A',
  background: '#0A0A0A',
};

// Task priorities
type TaskPriority = 'low' | 'medium' | 'high' | 'critical';
type TaskStatus = 'backlog' | 'todo' | 'in_progress' | 'review' | 'done';

interface KanbanTask {
  id: string;
  title: string;
  description?: string;
  status: TaskStatus;
  priority: TaskPriority;
  assignee?: string;
  dueDate?: string;
  tags?: string[];
  subtasks?: { id: string; title: string; completed: boolean }[];
  createdAt: string;
  updatedAt: string;
}

interface KanbanColumn {
  id: TaskStatus;
  title: string;
  icon: string;
  color: string;
  wipLimit?: number;
}

// Column definitions
const COLUMNS: KanbanColumn[] = [
  { id: 'backlog', title: 'Backlog', icon: 'Inbox', color: COLORS.muted },
  { id: 'todo', title: 'To Do', icon: 'ListTodo', color: COLORS.warning },
  { id: 'in_progress', title: 'In Progress', icon: 'PlayCircle', color: COLORS.primary, wipLimit: 5 },
  { id: 'review', title: 'Review', icon: 'Eye', color: COLORS.secondary, wipLimit: 3 },
  { id: 'done', title: 'Done', icon: 'CheckCircle', color: COLORS.accent },
];

// Priority colors
const PRIORITY_COLORS: Record<TaskPriority, string> = {
  low: COLORS.muted,
  medium: COLORS.warning,
  high: '#FF8C00',
  critical: COLORS.error,
};

// Dynamic icon component
function DynamicIcon({ name, size = 16, color = COLORS.foreground }: { 
  name: string; 
  size?: number; 
  color?: string;
}) {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const IconComponent = (LucideIcons as any)[name] || LucideIcons.Circle;
  return <IconComponent size={size} color={color} />;
}

// Task Card Component
interface TaskCardProps {
  task: KanbanTask;
  onEdit?: (task: KanbanTask) => void;
  onDelete?: (id: string) => void;
  isDragging?: boolean;
}

function TaskCard({ task, onEdit, onDelete, isDragging }: TaskCardProps) {
  const [showActions, setShowActions] = useState(false);
  const completedSubtasks = task.subtasks?.filter(s => s.completed).length || 0;
  const totalSubtasks = task.subtasks?.length || 0;

  return (
    <motion.div
      layout
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      exit={{ opacity: 0, scale: 0.95 }}
      whileHover={{ scale: 1.02 }}
      className={cn(
        "p-3 rounded-lg border border-[#2A2A2A] bg-[#1A1A1A] cursor-grab active:cursor-grabbing",
        "hover:border-[#00D4FF]/30 transition-colors",
        isDragging && "shadow-lg shadow-[#00D4FF]/20 border-[#00D4FF]/50"
      )}
      onMouseEnter={() => setShowActions(true)}
      onMouseLeave={() => setShowActions(false)}
    >
      {/* Header */}
      <div className="flex items-start justify-between gap-2 mb-2">
        <div className="flex items-center gap-2">
          {/* Priority indicator */}
          <div 
            className="w-2 h-2 rounded-full"
            style={{ backgroundColor: PRIORITY_COLORS[task.priority] }}
          />
          <span className="text-xs text-[#9BA1A6] font-mono">#{task.id.slice(0, 8)}</span>
        </div>
        
        {/* Action buttons */}
        <AnimatePresence>
          {showActions && (
            <motion.div
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              exit={{ opacity: 0 }}
              className="flex items-center gap-1"
            >
              <button
                onClick={() => onEdit?.(task)}
                className="p-1 rounded hover:bg-[rgba(255,255,255,0.1)] transition-colors"
              >
                <LucideIcons.Edit2 size={12} color={COLORS.muted} />
              </button>
              <button
                onClick={() => onDelete?.(task.id)}
                className="p-1 rounded hover:bg-[rgba(255,255,255,0.1)] transition-colors"
              >
                <LucideIcons.Trash2 size={12} color={COLORS.error} />
              </button>
            </motion.div>
          )}
        </AnimatePresence>
      </div>

      {/* Title */}
      <h4 className="text-sm font-medium text-[#ECEDEE] mb-2 line-clamp-2">
        {task.title}
      </h4>

      {/* Description */}
      {task.description && (
        <p className="text-xs text-[#9BA1A6] mb-3 line-clamp-2">
          {task.description}
        </p>
      )}

      {/* Tags */}
      {task.tags && task.tags.length > 0 && (
        <div className="flex flex-wrap gap-1 mb-3">
          {task.tags.slice(0, 3).map(tag => (
            <span
              key={tag}
              className="px-1.5 py-0.5 text-[10px] rounded bg-[rgba(0,212,255,0.1)] text-[#00D4FF]"
            >
              {tag}
            </span>
          ))}
          {task.tags.length > 3 && (
            <span className="px-1.5 py-0.5 text-[10px] rounded bg-[rgba(255,255,255,0.05)] text-[#9BA1A6]">
              +{task.tags.length - 3}
            </span>
          )}
        </div>
      )}

      {/* Footer */}
      <div className="flex items-center justify-between pt-2 border-t border-[#2A2A2A]">
        {/* Subtasks progress */}
        {totalSubtasks > 0 && (
          <div className="flex items-center gap-1.5 text-[10px] text-[#9BA1A6]">
            <LucideIcons.CheckSquare size={12} />
            <span>{completedSubtasks}/{totalSubtasks}</span>
          </div>
        )}

        {/* Due date */}
        {task.dueDate && (
          <div className="flex items-center gap-1.5 text-[10px] text-[#9BA1A6]">
            <LucideIcons.Calendar size={12} />
            <span>{new Date(task.dueDate).toLocaleDateString()}</span>
          </div>
        )}

        {/* Assignee */}
        {task.assignee && (
          <div className="w-5 h-5 rounded-full bg-[#2A2A2A] flex items-center justify-center">
            <span className="text-[10px] text-[#ECEDEE]">
              {task.assignee.charAt(0).toUpperCase()}
            </span>
          </div>
        )}
      </div>
    </motion.div>
  );
}

// Column Component
interface ColumnProps {
  column: KanbanColumn;
  tasks: KanbanTask[];
  onTaskMove?: (taskId: string, newStatus: TaskStatus) => void;
  onTaskEdit?: (task: KanbanTask) => void;
  onTaskDelete?: (id: string) => void;
  onAddTask?: (status: TaskStatus) => void;
}

function Column({ column, tasks, onTaskMove, onTaskEdit, onTaskDelete, onAddTask }: ColumnProps) {
  const [isDragOver, setIsDragOver] = useState(false);
  const isOverWipLimit = column.wipLimit && tasks.length >= column.wipLimit;

  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    setIsDragOver(true);
  };

  const handleDragLeave = () => {
    setIsDragOver(false);
  };

  const handleDrop = (e: React.DragEvent) => {
    e.preventDefault();
    setIsDragOver(false);
    const taskId = e.dataTransfer.getData('taskId');
    if (taskId) {
      onTaskMove?.(taskId, column.id);
    }
  };

  const handleDragStart = (e: React.DragEvent, taskId: string) => {
    e.dataTransfer.setData('taskId', taskId);
  };

  return (
    <div
      className={cn(
        "flex-1 min-w-[280px] max-w-[320px] flex flex-col",
        "rounded-xl border border-[#2A2A2A] bg-[#0F0F0F]",
        isDragOver && "border-[#00D4FF]/50 bg-[#00D4FF]/5"
      )}
      onDragOver={handleDragOver}
      onDragLeave={handleDragLeave}
      onDrop={handleDrop}
    >
      {/* Column Header */}
      <div className="flex items-center justify-between p-3 border-b border-[#2A2A2A]">
        <div className="flex items-center gap-2">
          <DynamicIcon name={column.icon} size={18} color={column.color} />
          <h3 className="font-medium text-[#ECEDEE]">{column.title}</h3>
          <span className="px-1.5 py-0.5 text-xs rounded bg-[rgba(255,255,255,0.1)] text-[#9BA1A6]">
            {tasks.length}
            {column.wipLimit && `/${column.wipLimit}`}
          </span>
        </div>
        <button
          onClick={() => onAddTask?.(column.id)}
          className="p-1 rounded hover:bg-[rgba(255,255,255,0.1)] transition-colors"
        >
          <LucideIcons.Plus size={16} color={COLORS.muted} />
        </button>
      </div>

      {/* WIP Limit Warning */}
      {isOverWipLimit && (
        <div className="px-3 py-2 bg-[rgba(255,140,0,0.1)] border-b border-[rgba(255,140,0,0.2)]">
          <div className="flex items-center gap-2 text-[10px] text-[#FF8C00]">
            <LucideIcons.AlertTriangle size={12} />
            <span>WIP limit reached</span>
          </div>
        </div>
      )}

      {/* Tasks */}
      <div className="flex-1 p-2 space-y-2 overflow-y-auto scrollbar-hide">
        <AnimatePresence mode="popLayout">
          {tasks.map(task => (
            <div
              key={task.id}
              draggable
              onDragStart={(e) => handleDragStart(e, task.id)}
            >
              <TaskCard
                task={task}
                onEdit={onTaskEdit}
                onDelete={onTaskDelete}
              />
            </div>
          ))}
        </AnimatePresence>

        {tasks.length === 0 && (
          <div className="flex flex-col items-center justify-center py-8 text-[#9BA1A6]">
            <LucideIcons.Inbox size={24} className="mb-2 opacity-50" />
            <p className="text-xs">No tasks</p>
          </div>
        )}
      </div>
    </div>
  );
}

// Main Kanban Board Component
interface KanbanBoardProps {
  initialTasks?: KanbanTask[];
  onTasksChange?: (tasks: KanbanTask[]) => void;
}

export function KanbanBoard({ initialTasks = [], onTasksChange }: KanbanBoardProps) {
  const [tasks, setTasks] = useState<KanbanTask[]>(initialTasks.length > 0 ? initialTasks : SAMPLE_TASKS);
  const [showNewTaskModal, setShowNewTaskModal] = useState(false);
  const [newTaskStatus, setNewTaskStatus] = useState<TaskStatus>('backlog');
  const [searchQuery, setSearchQuery] = useState('');
  const [filterPriority, setFilterPriority] = useState<TaskPriority | 'all'>('all');

  const handleTaskMove = useCallback((taskId: string, newStatus: TaskStatus) => {
    setTasks(prev => {
      const updated = prev.map(task =>
        task.id === taskId
          ? { ...task, status: newStatus, updatedAt: new Date().toISOString() }
          : task
      );
      onTasksChange?.(updated);
      return updated;
    });
  }, [onTasksChange]);

  const handleTaskDelete = useCallback((taskId: string) => {
    setTasks(prev => {
      const updated = prev.filter(task => task.id !== taskId);
      onTasksChange?.(updated);
      return updated;
    });
  }, [onTasksChange]);

  const handleAddTask = useCallback((status: TaskStatus) => {
    setNewTaskStatus(status);
    setShowNewTaskModal(true);
  }, []);

  const handleCreateTask = useCallback((title: string, description: string, priority: TaskPriority) => {
    const newTask: KanbanTask = {
      id: `task-${Date.now()}`,
      title,
      description,
      status: newTaskStatus,
      priority,
      createdAt: new Date().toISOString(),
      updatedAt: new Date().toISOString(),
    };
    setTasks(prev => {
      const updated = [...prev, newTask];
      onTasksChange?.(updated);
      return updated;
    });
    setShowNewTaskModal(false);
  }, [newTaskStatus, onTasksChange]);

  // Filter tasks
  const filteredTasks = tasks.filter(task => {
    const matchesSearch = searchQuery === '' || 
      task.title.toLowerCase().includes(searchQuery.toLowerCase()) ||
      task.description?.toLowerCase().includes(searchQuery.toLowerCase());
    const matchesPriority = filterPriority === 'all' || task.priority === filterPriority;
    return matchesSearch && matchesPriority;
  });

  // Group tasks by status
  const tasksByStatus = COLUMNS.reduce((acc, column) => {
    acc[column.id] = filteredTasks.filter(task => task.status === column.id);
    return acc;
  }, {} as Record<TaskStatus, KanbanTask[]>);

  return (
    <div className="flex flex-col h-full bg-[#0A0A0A]">
      {/* Header */}
      <div className="flex items-center justify-between p-4 border-b border-[#2A2A2A]">
        <div className="flex items-center gap-4">
          <h2 className="text-xl font-semibold text-[#ECEDEE]">Kanban Board</h2>
          <span className="px-2 py-0.5 text-xs rounded bg-[rgba(0,212,255,0.1)] text-[#00D4FF]">
            {tasks.length} tasks
          </span>
        </div>

        <div className="flex items-center gap-3">
          {/* Search */}
          <div className="flex items-center gap-2 px-3 py-1.5 rounded-lg border border-[#2A2A2A] bg-[#1A1A1A]">
            <LucideIcons.Search size={14} color={COLORS.muted} />
            <input
              type="text"
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              placeholder="Search tasks..."
              className="bg-transparent text-sm text-[#ECEDEE] placeholder-[#9BA1A6] outline-none w-40"
            />
          </div>

          {/* Priority Filter */}
          <select
            value={filterPriority}
            onChange={(e) => setFilterPriority(e.target.value as TaskPriority | 'all')}
            className="px-3 py-1.5 rounded-lg border border-[#2A2A2A] bg-[#1A1A1A] text-sm text-[#ECEDEE] outline-none"
          >
            <option value="all">All Priorities</option>
            <option value="critical">Critical</option>
            <option value="high">High</option>
            <option value="medium">Medium</option>
            <option value="low">Low</option>
          </select>

          {/* New Task Button */}
          <button
            onClick={() => handleAddTask('backlog')}
            className="flex items-center gap-2 px-3 py-1.5 rounded-lg bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] text-white text-sm font-medium hover:opacity-90 transition-opacity"
          >
            <LucideIcons.Plus size={16} />
            New Task
          </button>
        </div>
      </div>

      {/* Board */}
      <div className="flex-1 overflow-x-auto p-4">
        <div className="flex gap-4 h-full">
          {COLUMNS.map(column => (
            <Column
              key={column.id}
              column={column}
              tasks={tasksByStatus[column.id] || []}
              onTaskMove={handleTaskMove}
              onTaskEdit={(task) => console.log('Edit task:', task)}
              onTaskDelete={handleTaskDelete}
              onAddTask={handleAddTask}
            />
          ))}
        </div>
      </div>

      {/* New Task Modal */}
      <AnimatePresence>
        {showNewTaskModal && (
          <NewTaskModal
            status={newTaskStatus}
            onClose={() => setShowNewTaskModal(false)}
            onCreate={handleCreateTask}
          />
        )}
      </AnimatePresence>
    </div>
  );
}

// New Task Modal
interface NewTaskModalProps {
  status: TaskStatus;
  onClose: () => void;
  onCreate: (title: string, description: string, priority: TaskPriority) => void;
}

function NewTaskModal({ status, onClose, onCreate }: NewTaskModalProps) {
  const [title, setTitle] = useState('');
  const [description, setDescription] = useState('');
  const [priority, setPriority] = useState<TaskPriority>('medium');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (title.trim()) {
      onCreate(title.trim(), description.trim(), priority);
    }
  };

  return (
    <>
      <motion.div
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        exit={{ opacity: 0 }}
        className="fixed inset-0 bg-black/60 z-40"
        onClick={onClose}
      />
      <motion.div
        initial={{ opacity: 0, scale: 0.95 }}
        animate={{ opacity: 1, scale: 1 }}
        exit={{ opacity: 0, scale: 0.95 }}
        className="fixed left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2 w-full max-w-md z-50"
      >
        <form
          onSubmit={handleSubmit}
          className="bg-[#1A1A1A] border border-[#2A2A2A] rounded-xl p-6 shadow-2xl"
        >
          <h3 className="text-lg font-semibold text-[#ECEDEE] mb-4">Create New Task</h3>

          <div className="space-y-4">
            <div>
              <label className="block text-xs text-[#9BA1A6] mb-1">Title</label>
              <input
                type="text"
                value={title}
                onChange={(e) => setTitle(e.target.value)}
                placeholder="Enter task title..."
                className="w-full px-3 py-2 rounded-lg border border-[#2A2A2A] bg-[#0A0A0A] text-[#ECEDEE] placeholder-[#9BA1A6] outline-none focus:border-[#00D4FF]/50"
                autoFocus
              />
            </div>

            <div>
              <label className="block text-xs text-[#9BA1A6] mb-1">Description</label>
              <textarea
                value={description}
                onChange={(e) => setDescription(e.target.value)}
                placeholder="Enter task description..."
                rows={3}
                className="w-full px-3 py-2 rounded-lg border border-[#2A2A2A] bg-[#0A0A0A] text-[#ECEDEE] placeholder-[#9BA1A6] outline-none focus:border-[#00D4FF]/50 resize-none"
              />
            </div>

            <div>
              <label className="block text-xs text-[#9BA1A6] mb-1">Priority</label>
              <select
                value={priority}
                onChange={(e) => setPriority(e.target.value as TaskPriority)}
                className="w-full px-3 py-2 rounded-lg border border-[#2A2A2A] bg-[#0A0A0A] text-[#ECEDEE] outline-none focus:border-[#00D4FF]/50"
              >
                <option value="low">Low</option>
                <option value="medium">Medium</option>
                <option value="high">High</option>
                <option value="critical">Critical</option>
              </select>
            </div>

            <div className="text-xs text-[#9BA1A6]">
              Status: <span className="text-[#00D4FF]">{status.replace('_', ' ')}</span>
            </div>
          </div>

          <div className="flex items-center justify-end gap-2 mt-6">
            <button
              type="button"
              onClick={onClose}
              className="px-4 py-2 rounded-lg text-[#9BA1A6] hover:bg-[rgba(255,255,255,0.05)] transition-colors"
            >
              Cancel
            </button>
            <button
              type="submit"
              disabled={!title.trim()}
              className="px-4 py-2 rounded-lg bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] text-white font-medium hover:opacity-90 transition-opacity disabled:opacity-50 disabled:cursor-not-allowed"
            >
              Create Task
            </button>
          </div>
        </form>
      </motion.div>
    </>
  );
}

// Sample tasks for demo
const SAMPLE_TASKS: KanbanTask[] = [
  {
    id: 'task-001',
    title: 'Implement double minimize sidebar',
    description: 'Add functionality to collapse both icon rail and detail panel',
    status: 'done',
    priority: 'high',
    tags: ['frontend', 'ui'],
    createdAt: '2024-01-10T10:00:00Z',
    updatedAt: '2024-01-14T15:00:00Z',
  },
  {
    id: 'task-002',
    title: 'Integrate LocalAI service',
    description: 'Connect frontend to LocalAI for LLM completions',
    status: 'in_progress',
    priority: 'high',
    assignee: 'dev',
    tags: ['ai', 'integration'],
    createdAt: '2024-01-12T09:00:00Z',
    updatedAt: '2024-01-14T12:00:00Z',
  },
  {
    id: 'task-003',
    title: 'Setup AGiXT agents',
    description: 'Configure AGiXT sandbox and agent chains',
    status: 'in_progress',
    priority: 'medium',
    tags: ['ai', 'backend'],
    createdAt: '2024-01-13T14:00:00Z',
    updatedAt: '2024-01-14T10:00:00Z',
  },
  {
    id: 'task-004',
    title: 'Add web browser panel',
    description: 'Implement embedded browser in sidebar',
    status: 'review',
    priority: 'medium',
    tags: ['frontend', 'feature'],
    createdAt: '2024-01-11T11:00:00Z',
    updatedAt: '2024-01-14T14:00:00Z',
  },
  {
    id: 'task-005',
    title: 'Cross-platform sync testing',
    description: 'Verify state sync between web and mobile',
    status: 'todo',
    priority: 'critical',
    tags: ['testing', 'sync'],
    createdAt: '2024-01-14T08:00:00Z',
    updatedAt: '2024-01-14T08:00:00Z',
  },
  {
    id: 'task-006',
    title: 'Component library documentation',
    description: 'Document all shared components',
    status: 'backlog',
    priority: 'low',
    tags: ['docs'],
    createdAt: '2024-01-09T10:00:00Z',
    updatedAt: '2024-01-09T10:00:00Z',
  },
  {
    id: 'task-007',
    title: 'Configuration audit',
    description: 'Review all config files for issues',
    status: 'todo',
    priority: 'high',
    tags: ['config', 'security'],
    createdAt: '2024-01-14T09:00:00Z',
    updatedAt: '2024-01-14T09:00:00Z',
  },
];

export default KanbanBoard;
