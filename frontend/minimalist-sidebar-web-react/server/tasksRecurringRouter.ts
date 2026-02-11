import { protectedProcedure, router } from "./_core/trpc";
import { z } from "zod";
import * as db from "./db";

// Recurrence pattern schema
const recurrencePatternSchema = z.enum([
  "daily",
  "weekly",
  "biweekly",
  "monthly",
  "quarterly",
  "yearly",
  "weekdays", // Monday-Friday
  "weekends", // Saturday-Sunday
  "custom", // Custom cron pattern
]);

export const tasksRecurringRouter = router({
  // Create a recurring task
  create: protectedProcedure
    .input(z.object({
      title: z.string(),
      description: z.string().optional(),
      priority: z.enum(["low", "medium", "high", "urgent"]).optional(),
      recurrencePattern: recurrencePatternSchema,
      customCron: z.string().optional(), // For custom patterns
      recurrenceEndDate: z.date().optional(),
      assignee: z.string().optional(),
      tags: z.array(z.string()).optional(),
      estimatedTime: z.number().optional(),
    }))
    .mutation(async ({ input, ctx }) => {
      const recurrencePattern = input.recurrencePattern === "custom" && input.customCron
        ? `custom:${input.customCron}`
        : input.recurrencePattern;

      const task = await db.createTask({
        userId: ctx.user.id,
        title: input.title,
        description: input.description,
        status: "todo",
        priority: input.priority || "medium",
        assignee: input.assignee,
        tags: input.tags || [],
        estimatedTime: input.estimatedTime,
        progress: 0,
        starred: false,
        isRecurring: true,
        recurrencePattern,
        recurrenceEndDate: input.recurrenceEndDate,
        hasInProgressAttempt: false,
        lastAttemptFailed: false,
      });

      // Create initial activity
      await db.createTaskActivity({
        taskId: task.id,
        type: "created",
        user: ctx.user.name || "User",
        description: `created recurring task (${input.recurrencePattern})`,
      });

      return task;
    }),

  // Get all recurring tasks
  list: protectedProcedure.query(async ({ ctx }) => {
    const tasks = await db.getTasksByUserId(ctx.user.id);
    return tasks.filter(t => t.isRecurring && !t.parentRecurringTaskId);
  }),

  // Generate next instance of a recurring task
  generateNext: protectedProcedure
    .input(z.object({
      recurringTaskId: z.number(),
    }))
    .mutation(async ({ input, ctx }) => {
      const recurringTask = await db.getTaskById(input.recurringTaskId);

      if (!recurringTask || recurringTask.userId !== ctx.user.id || !recurringTask.isRecurring) {
        throw new Error("Recurring task not found");
      }

      // Check if recurrence has ended
      if (recurringTask.recurrenceEndDate && new Date(recurringTask.recurrenceEndDate) < new Date()) {
        throw new Error("Recurrence period has ended");
      }

      // Calculate next due date based on pattern
      const nextDueDate = calculateNextDueDate(recurringTask.recurrencePattern || "daily");

      // Create new instance
      const newTask = await db.createTask({
        userId: ctx.user.id,
        title: recurringTask.title,
        description: recurringTask.description,
        status: "todo",
        priority: recurringTask.priority,
        assignee: recurringTask.assignee,
        dueDate: nextDueDate,
        tags: recurringTask.tags || [],
        estimatedTime: recurringTask.estimatedTime,
        progress: 0,
        starred: false,
        parentRecurringTaskId: recurringTask.id,
        hasInProgressAttempt: false,
        lastAttemptFailed: false,
      });

      // Create activity
      await db.createTaskActivity({
        taskId: newTask.id,
        type: "created",
        user: "System",
        description: `auto-generated from recurring task`,
      });

      return newTask;
    }),

  // Update recurring task
  update: protectedProcedure
    .input(z.object({
      id: z.number(),
      title: z.string().optional(),
      description: z.string().optional(),
      priority: z.enum(["low", "medium", "high", "urgent"]).optional(),
      recurrencePattern: recurrencePatternSchema.optional(),
      customCron: z.string().optional(),
      recurrenceEndDate: z.date().optional(),
      assignee: z.string().optional(),
      tags: z.array(z.string()).optional(),
      estimatedTime: z.number().optional(),
    }))
    .mutation(async ({ input, ctx }) => {
      const { id, customCron, ...data } = input;

      const existingTask = await db.getTaskById(id);
      if (!existingTask || existingTask.userId !== ctx.user.id || !existingTask.isRecurring) {
        throw new Error("Recurring task not found");
      }

      const updateData: any = { ...data };

      if (data.recurrencePattern === "custom" && customCron) {
        updateData.recurrencePattern = `custom:${customCron}`;
      }

      const task = await db.updateTask(id, ctx.user.id, updateData);

      // Create activity
      await db.createTaskActivity({
        taskId: id,
        type: "updated",
        user: ctx.user.name || "User",
        description: "updated recurring task settings",
      });

      return task;
    }),

  // Delete recurring task (and optionally all instances)
  delete: protectedProcedure
    .input(z.object({
      id: z.number(),
      deleteInstances: z.boolean().default(false),
    }))
    .mutation(async ({ input, ctx }) => {
      const recurringTask = await db.getTaskById(input.id);

      if (!recurringTask || recurringTask.userId !== ctx.user.id || !recurringTask.isRecurring) {
        throw new Error("Recurring task not found");
      }

      // Delete all instances if requested
      if (input.deleteInstances) {
        const allTasks = await db.getTasksByUserId(ctx.user.id);
        const instances = allTasks.filter(t => t.parentRecurringTaskId === input.id);

        await Promise.all(
          instances.map(instance => db.deleteTask(instance.id, ctx.user.id))
        );
      }

      // Delete the recurring task itself
      const success = await db.deleteTask(input.id, ctx.user.id);

      if (!success) {
        throw new Error("Failed to delete recurring task");
      }

      return { success: true, deletedInstances: input.deleteInstances };
    }),

  // Get instances of a recurring task
  getInstances: protectedProcedure
    .input(z.object({
      recurringTaskId: z.number(),
    }))
    .query(async ({ input, ctx }) => {
      const allTasks = await db.getTasksByUserId(ctx.user.id);
      return allTasks.filter(t => t.parentRecurringTaskId === input.recurringTaskId);
    }),

  // Pause/resume recurring task
  togglePause: protectedProcedure
    .input(z.object({
      id: z.number(),
    }))
    .mutation(async ({ input, ctx }) => {
      const task = await db.getTaskById(input.id);

      if (!task || task.userId !== ctx.user.id || !task.isRecurring) {
        throw new Error("Recurring task not found");
      }

      // Use a custom field or status to track pause state
      // For now, we'll set recurrenceEndDate to today to "pause"
      const isPaused = task.recurrenceEndDate && new Date(task.recurrenceEndDate) <= new Date();

      const updateData = isPaused
        ? { recurrenceEndDate: null } // Resume
        : { recurrenceEndDate: new Date() }; // Pause

      await db.updateTask(input.id, ctx.user.id, updateData);

      // Create activity
      await db.createTaskActivity({
        taskId: input.id,
        type: "updated",
        user: ctx.user.name || "User",
        description: isPaused ? "resumed recurring task" : "paused recurring task",
      });

      return { paused: !isPaused };
    }),
});

// Helper function to calculate next due date based on recurrence pattern
function calculateNextDueDate(pattern: string): Date {
  const now = new Date();
  const nextDate = new Date(now);

  if (pattern.startsWith("custom:")) {
    // For custom cron patterns, we'd need a cron parser
    // For now, default to next day
    nextDate.setDate(nextDate.getDate() + 1);
    return nextDate;
  }

  switch (pattern) {
    case "daily":
      nextDate.setDate(nextDate.getDate() + 1);
      break;
    case "weekly":
      nextDate.setDate(nextDate.getDate() + 7);
      break;
    case "biweekly":
      nextDate.setDate(nextDate.getDate() + 14);
      break;
    case "monthly":
      nextDate.setMonth(nextDate.getMonth() + 1);
      break;
    case "quarterly":
      nextDate.setMonth(nextDate.getMonth() + 3);
      break;
    case "yearly":
      nextDate.setFullYear(nextDate.getFullYear() + 1);
      break;
    case "weekdays":
      // Next weekday
      nextDate.setDate(nextDate.getDate() + 1);
      while (nextDate.getDay() === 0 || nextDate.getDay() === 6) {
        nextDate.setDate(nextDate.getDate() + 1);
      }
      break;
    case "weekends":
      // Next weekend day
      nextDate.setDate(nextDate.getDate() + 1);
      while (nextDate.getDay() !== 0 && nextDate.getDay() !== 6) {
        nextDate.setDate(nextDate.getDate() + 1);
      }
      break;
    default:
      nextDate.setDate(nextDate.getDate() + 1);
  }

  return nextDate;
}
