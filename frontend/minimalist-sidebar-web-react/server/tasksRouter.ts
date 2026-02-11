import { protectedProcedure, router } from "./_core/trpc";
import { z } from "zod";
import * as db from "./db";
import { tasksAIRouter } from "./tasksAIRouter";
import { tasksRecurringRouter } from "./tasksRecurringRouter";

// Zod schemas for validation
const taskStatusSchema = z.enum(["todo", "in_progress", "review", "done", "cancelled"]);
const taskPrioritySchema = z.enum(["low", "medium", "high", "urgent"]);

const createTaskSchema = z.object({
  title: z.string().min(1).max(255),
  description: z.string().optional(),
  status: taskStatusSchema.optional(),
  priority: taskPrioritySchema.optional(),
  assignee: z.string().optional(),
  dueDate: z.date().optional(),
  tags: z.array(z.string()).optional(),
  estimatedTime: z.number().optional(),
  progress: z.number().min(0).max(100).optional(),
  starred: z.boolean().optional(),
  parentTaskId: z.number().optional(),
  sharedTaskId: z.string().optional(),
  projectId: z.string().optional(),
  hasInProgressAttempt: z.boolean().optional(),
  lastAttemptFailed: z.boolean().optional(),
  executor: z.string().optional(),
});

const updateTaskSchema = createTaskSchema.partial().extend({
  id: z.number(),
});

const taskCommentSchema = z.object({
  taskId: z.number(),
  author: z.string(),
  content: z.string(),
  avatar: z.string().optional(),
});

const taskActivitySchema = z.object({
  taskId: z.number(),
  type: z.enum(["created", "updated", "commented", "status_changed", "assigned"]),
  user: z.string(),
  description: z.string(),
});

const taskTimeEntrySchema = z.object({
  taskId: z.number(),
  duration: z.number(),
  description: z.string(),
});

const taskDependencySchema = z.object({
  taskId: z.number(),
  dependsOnTaskId: z.number(),
});

export const tasksRouter = router({
  // AI-powered features
  ai: tasksAIRouter,
  // Recurring tasks
  recurring: tasksRecurringRouter,

  // ==================== TASK CRUD ====================
  
  // Get all tasks for the current user
  list: protectedProcedure.query(async ({ ctx }) => {
    const tasks = await db.getTasksByUserId(ctx.user.id);
    
    // Fetch related data for each task
    const tasksWithRelations = await Promise.all(
      tasks.map(async (task) => {
        const [comments, activities, timeEntries, dependencies, subtasks] = await Promise.all([
          db.getTaskComments(task.id),
          db.getTaskActivities(task.id),
          db.getTaskTimeEntries(task.id),
          db.getTaskDependencies(task.id),
          db.getSubtasks(task.id),
        ]);

        return {
          ...task,
          comments,
          activities,
          timeEntries,
          dependencies: dependencies.map(d => d.dependsOnTaskId),
          subtasks,
        };
      })
    );

    return tasksWithRelations;
  }),

  // Get a single task by ID
  getById: protectedProcedure
    .input(z.object({ id: z.number() }))
    .query(async ({ input, ctx }) => {
      const task = await db.getTaskById(input.id);
      
      if (!task || task.userId !== ctx.user.id) {
        throw new Error("Task not found");
      }

      const [comments, activities, timeEntries, dependencies, subtasks] = await Promise.all([
        db.getTaskComments(task.id),
        db.getTaskActivities(task.id),
        db.getTaskTimeEntries(task.id),
        db.getTaskDependencies(task.id),
        db.getSubtasks(task.id),
      ]);

      return {
        ...task,
        comments,
        activities,
        timeEntries,
        dependencies: dependencies.map(d => d.dependsOnTaskId),
        subtasks,
      };
    }),

  // Create a new task
  create: protectedProcedure
    .input(createTaskSchema)
    .mutation(async ({ input, ctx }) => {
      // Generate unique taskId
      const existingTasks = await db.getTasksByUserId(ctx.user.id);
      const taskNumber = existingTasks.length + 1;
      const taskId = `TASK-${taskNumber}`;
      
      const task = await db.createTask({
        taskId,
        ...input,
        userId: ctx.user.id,
        tags: input.tags || [],
        progress: input.progress || 0,
        starred: input.starred || false,
        hasInProgressAttempt: input.hasInProgressAttempt || false,
        lastAttemptFailed: input.lastAttemptFailed || false,
      });

      // Create initial activity
      await db.createTaskActivity({
        taskId: task.id,
        type: "created",
        user: ctx.user.name || "User",
        description: "created this task",
      });

      return task;
    }),

  // Update an existing task
  update: protectedProcedure
    .input(updateTaskSchema)
    .mutation(async ({ input, ctx }) => {
      const { id, ...data } = input;
      
      const existingTask = await db.getTaskById(id);
      if (!existingTask || existingTask.userId !== ctx.user.id) {
        throw new Error("Task not found");
      }

      const task = await db.updateTask(id, ctx.user.id, data);

      // Create update activity
      await db.createTaskActivity({
        taskId: id,
        type: "updated",
        user: ctx.user.name || "User",
        description: "updated this task",
      });

      // If status changed, create status_changed activity
      if (data.status && data.status !== existingTask.status) {
        await db.createTaskActivity({
          taskId: id,
          type: "status_changed",
          user: ctx.user.name || "User",
          description: `moved from ${existingTask.status} to ${data.status}`,
        });
      }

      return task;
    }),

  // Delete a task
  delete: protectedProcedure
    .input(z.object({ id: z.number() }))
    .mutation(async ({ input, ctx }) => {
      const success = await db.deleteTask(input.id, ctx.user.id);
      
      if (!success) {
        throw new Error("Task not found or already deleted");
      }

      return { success: true };
    }),

  // Get subtasks for a parent task
  getSubtasks: protectedProcedure
    .input(z.object({ parentTaskId: z.number() }))
    .query(async ({ input, ctx }) => {
      const subtasks = await db.getSubtasks(input.parentTaskId);
      
      // Filter to only return subtasks owned by the current user
      return subtasks.filter(task => task.userId === ctx.user.id);
    }),

  // ==================== TASK COMMENTS ====================

  // Add a comment to a task
  addComment: protectedProcedure
    .input(taskCommentSchema)
    .mutation(async ({ input, ctx }) => {
      const task = await db.getTaskById(input.taskId);
      
      if (!task || task.userId !== ctx.user.id) {
        throw new Error("Task not found");
      }

      const comment = await db.createTaskComment({
        ...input,
        author: input.author || ctx.user.name || "User",
        avatar: input.avatar || ctx.user.avatarUrl,
      });

      // Create comment activity
      await db.createTaskActivity({
        taskId: input.taskId,
        type: "commented",
        user: ctx.user.name || "User",
        description: "added a comment",
      });

      return comment;
    }),

  // Get comments for a task
  getComments: protectedProcedure
    .input(z.object({ taskId: z.number() }))
    .query(async ({ input, ctx }) => {
      const task = await db.getTaskById(input.taskId);
      
      if (!task || task.userId !== ctx.user.id) {
        throw new Error("Task not found");
      }

      return db.getTaskComments(input.taskId);
    }),

  // Delete a comment
  deleteComment: protectedProcedure
    .input(z.object({ id: z.number() }))
    .mutation(async ({ input }) => {
      const success = await db.deleteTaskComment(input.id);
      
      if (!success) {
        throw new Error("Comment not found");
      }

      return { success: true };
    }),

  // ==================== TASK ACTIVITIES ====================

  // Get activities for a task
  getActivities: protectedProcedure
    .input(z.object({ taskId: z.number() }))
    .query(async ({ input, ctx }) => {
      const task = await db.getTaskById(input.taskId);
      
      if (!task || task.userId !== ctx.user.id) {
        throw new Error("Task not found");
      }

      return db.getTaskActivities(input.taskId);
    }),

  // ==================== TASK TIME ENTRIES ====================

  // Add a time entry to a task
  addTimeEntry: protectedProcedure
    .input(taskTimeEntrySchema)
    .mutation(async ({ input, ctx }) => {
      const task = await db.getTaskById(input.taskId);
      
      if (!task || task.userId !== ctx.user.id) {
        throw new Error("Task not found");
      }

      const entry = await db.createTaskTimeEntry(input);

      // Create activity for time tracking
      await db.createTaskActivity({
        taskId: input.taskId,
        type: "updated",
        user: ctx.user.name || "User",
        description: `logged ${input.duration} minutes`,
      });

      return entry;
    }),

  // Get time entries for a task
  getTimeEntries: protectedProcedure
    .input(z.object({ taskId: z.number() }))
    .query(async ({ input, ctx }) => {
      const task = await db.getTaskById(input.taskId);
      
      if (!task || task.userId !== ctx.user.id) {
        throw new Error("Task not found");
      }

      return db.getTaskTimeEntries(input.taskId);
    }),

  // Delete a time entry
  deleteTimeEntry: protectedProcedure
    .input(z.object({ id: z.number() }))
    .mutation(async ({ input }) => {
      const success = await db.deleteTaskTimeEntry(input.id);
      
      if (!success) {
        throw new Error("Time entry not found");
      }

      return { success: true };
    }),

  // ==================== TASK DEPENDENCIES ====================

  // Add a dependency to a task
  addDependency: protectedProcedure
    .input(taskDependencySchema)
    .mutation(async ({ input, ctx }) => {
      const task = await db.getTaskById(input.taskId);
      
      if (!task || task.userId !== ctx.user.id) {
        throw new Error("Task not found");
      }

      return db.createTaskDependency(input);
    }),

  // Get dependencies for a task
  getDependencies: protectedProcedure
    .input(z.object({ taskId: z.number() }))
    .query(async ({ input, ctx }) => {
      const task = await db.getTaskById(input.taskId);
      
      if (!task || task.userId !== ctx.user.id) {
        throw new Error("Task not found");
      }

      return db.getTaskDependencies(input.taskId);
    }),

  // Delete a dependency
  deleteDependency: protectedProcedure
    .input(z.object({ id: z.number() }))
    .mutation(async ({ input }) => {
      const success = await db.deleteTaskDependency(input.id);
      
      if (!success) {
        throw new Error("Dependency not found");
      }

      return { success: true };
    }),

  // ==================== BULK OPERATIONS ====================

  // Bulk update task status (for drag and drop)
  bulkUpdateStatus: protectedProcedure
    .input(z.object({
      taskIds: z.array(z.number()),
      status: taskStatusSchema,
    }))
    .mutation(async ({ input, ctx }) => {
      const updates = await Promise.all(
        input.taskIds.map(async (taskId) => {
          const task = await db.getTaskById(taskId);
          
          if (!task || task.userId !== ctx.user.id) {
            return null;
          }

          await db.updateTask(taskId, ctx.user.id, { status: input.status });
          
          await db.createTaskActivity({
            taskId,
            type: "status_changed",
            user: ctx.user.name || "User",
            description: `moved to ${input.status}`,
          });

          return taskId;
        })
      );

      return { updated: updates.filter(Boolean).length };
    }),

  // Get analytics for tasks
  getAnalytics: protectedProcedure.query(async ({ ctx }) => {
    const tasks = await db.getTasksByUserId(ctx.user.id);
    
    const total = tasks.length;
    const completed = tasks.filter(t => t.status === "done").length;
    const inProgress = tasks.filter(t => t.status === "in_progress").length;
    const cancelled = tasks.filter(t => t.status === "cancelled").length;
    
    const now = new Date();
    const overdue = tasks.filter(
      t => t.dueDate && new Date(t.dueDate) < now && t.status !== "done"
    ).length;

    const totalEstimated = tasks.reduce((sum, t) => sum + (t.estimatedTime || 0), 0);
    
    const priorityBreakdown = {
      urgent: tasks.filter(t => t.priority === "urgent").length,
      high: tasks.filter(t => t.priority === "high").length,
      medium: tasks.filter(t => t.priority === "medium").length,
      low: tasks.filter(t => t.priority === "low").length,
    };

    return {
      total,
      completed,
      inProgress,
      cancelled,
      overdue,
      completionRate: total > 0 ? Math.round((completed / total) * 100) : 0,
      totalEstimated,
      priorityBreakdown,
    };
  }),
});
