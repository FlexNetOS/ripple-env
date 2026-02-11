import { int, mysqlEnum, mysqlTable, text, timestamp, varchar, boolean, json } from "drizzle-orm/mysql-core";

/**
 * Core user table backing auth flow.
 * Columns use camelCase to match both database fields and generated types.
 */
export const users = mysqlTable("users", {
  id: int("id").autoincrement().primaryKey(),
  openId: varchar("openId", { length: 64 }).notNull().unique(),
  name: text("name"),
  email: varchar("email", { length: 320 }),
  avatarUrl: text("avatarUrl"),
  loginMethod: varchar("loginMethod", { length: 64 }),
  role: mysqlEnum("role", ["user", "admin"]).default("user").notNull(),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
  updatedAt: timestamp("updatedAt").defaultNow().onUpdateNow().notNull(),
  lastSignedIn: timestamp("lastSignedIn").defaultNow().notNull(),
});

export type User = typeof users.$inferSelect;
export type InsertUser = typeof users.$inferInsert;

/**
 * Files table for S3 file storage metadata
 */
export const files = mysqlTable("files", {
  id: int("id").autoincrement().primaryKey(),
  userId: int("userId").notNull(),
  fileName: varchar("fileName", { length: 255 }).notNull(),
  fileKey: varchar("fileKey", { length: 512 }).notNull(),
  url: text("url").notNull(),
  mimeType: varchar("mimeType", { length: 128 }),
  fileSize: int("fileSize"),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
  updatedAt: timestamp("updatedAt").defaultNow().onUpdateNow().notNull(),
});

export type File = typeof files.$inferSelect;
export type InsertFile = typeof files.$inferInsert;

/**
 * Chat conversations table
 */
export const conversations = mysqlTable("conversations", {
  id: int("id").autoincrement().primaryKey(),
  userId: int("userId").notNull(),
  title: varchar("title", { length: 255 }).default("New Conversation"),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
  updatedAt: timestamp("updatedAt").defaultNow().onUpdateNow().notNull(),
});

export type Conversation = typeof conversations.$inferSelect;
export type InsertConversation = typeof conversations.$inferInsert;

/**
 * Chat messages table
 */
export const messages = mysqlTable("messages", {
  id: int("id").autoincrement().primaryKey(),
  conversationId: int("conversationId").notNull(),
  role: mysqlEnum("role", ["user", "assistant", "system"]).notNull(),
  content: text("content").notNull(),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
});

export type Message = typeof messages.$inferSelect;
export type InsertMessage = typeof messages.$inferInsert;

/**
 * Notifications table for in-app alerts
 */
export const notifications = mysqlTable("notifications", {
  id: int("id").autoincrement().primaryKey(),
  userId: int("userId").notNull(),
  title: varchar("title", { length: 255 }).notNull(),
  content: text("content").notNull(),
  type: mysqlEnum("type", ["info", "success", "warning", "error"]).default("info").notNull(),
  isRead: boolean("isRead").default(false).notNull(),
  link: varchar("link", { length: 512 }),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
});

export type Notification = typeof notifications.$inferSelect;
export type InsertNotification = typeof notifications.$inferInsert;

/**
 * Subscription plans table
 */
export const subscriptionPlans = mysqlTable("subscription_plans", {
  id: int("id").autoincrement().primaryKey(),
  name: varchar("name", { length: 64 }).notNull(),
  description: text("description"),
  priceMonthly: int("priceMonthly").notNull(), // in cents
  priceYearly: int("priceYearly").notNull(), // in cents
  features: json("features").$type<string[]>(),
  stripePriceIdMonthly: varchar("stripePriceIdMonthly", { length: 128 }),
  stripePriceIdYearly: varchar("stripePriceIdYearly", { length: 128 }),
  isActive: boolean("isActive").default(true).notNull(),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
  updatedAt: timestamp("updatedAt").defaultNow().onUpdateNow().notNull(),
});

export type SubscriptionPlan = typeof subscriptionPlans.$inferSelect;
export type InsertSubscriptionPlan = typeof subscriptionPlans.$inferInsert;

/**
 * User subscriptions table
 */
export const subscriptions = mysqlTable("subscriptions", {
  id: int("id").autoincrement().primaryKey(),
  userId: int("userId").notNull(),
  planId: int("planId").notNull(),
  stripeCustomerId: varchar("stripeCustomerId", { length: 128 }),
  stripeSubscriptionId: varchar("stripeSubscriptionId", { length: 128 }),
  status: mysqlEnum("status", ["active", "canceled", "past_due", "trialing", "incomplete"]).default("incomplete").notNull(),
  billingCycle: mysqlEnum("billingCycle", ["monthly", "yearly"]).default("monthly").notNull(),
  currentPeriodStart: timestamp("currentPeriodStart"),
  currentPeriodEnd: timestamp("currentPeriodEnd"),
  cancelAtPeriodEnd: boolean("cancelAtPeriodEnd").default(false).notNull(),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
  updatedAt: timestamp("updatedAt").defaultNow().onUpdateNow().notNull(),
});

export type Subscription = typeof subscriptions.$inferSelect;
export type InsertSubscription = typeof subscriptions.$inferInsert;

/**
 * Payment history table
 */
export const payments = mysqlTable("payments", {
  id: int("id").autoincrement().primaryKey(),
  userId: int("userId").notNull(),
  subscriptionId: int("subscriptionId"),
  stripePaymentIntentId: varchar("stripePaymentIntentId", { length: 128 }),
  amount: int("amount").notNull(), // in cents
  currency: varchar("currency", { length: 3 }).default("usd").notNull(),
  status: mysqlEnum("status", ["succeeded", "pending", "failed", "refunded"]).default("pending").notNull(),
  description: text("description"),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
});

export type Payment = typeof payments.$inferSelect;
export type InsertPayment = typeof payments.$inferInsert;

/**
 * User settings/preferences table
 */
export const userSettings = mysqlTable("user_settings", {
  id: int("id").autoincrement().primaryKey(),
  userId: int("userId").notNull().unique(),
  emailNotifications: boolean("emailNotifications").default(true).notNull(),
  pushNotifications: boolean("pushNotifications").default(true).notNull(),
  theme: mysqlEnum("theme", ["light", "dark", "system"]).default("system").notNull(),
  language: varchar("language", { length: 10 }).default("en").notNull(),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
  updatedAt: timestamp("updatedAt").defaultNow().onUpdateNow().notNull(),
});

export type UserSettings = typeof userSettings.$inferSelect;
export type InsertUserSettings = typeof userSettings.$inferInsert;


/**
 * User favorites table for sidebar favorites persistence
 */
export const favorites = mysqlTable("favorites", {
  id: int("id").autoincrement().primaryKey(),
  userId: int("userId").notNull(),
  itemId: varchar("itemId", { length: 128 }).notNull(), // Sidebar item ID
  itemType: varchar("itemType", { length: 64 }).notNull(), // 'panel', 'section', 'item'
  title: varchar("title", { length: 255 }).notNull(),
  icon: varchar("icon", { length: 64 }),
  route: varchar("route", { length: 255 }),
  panelId: varchar("panelId", { length: 128 }), // Parent panel ID for context
  sortOrder: int("sortOrder").default(0).notNull(),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
});

export type Favorite = typeof favorites.$inferSelect;
export type InsertFavorite = typeof favorites.$inferInsert;

/**
 * Tasks table for task management with vibe-kanban features
 */
export const tasks = mysqlTable("tasks", {
  id: int("id").autoincrement().primaryKey(),
  taskId: varchar("taskId", { length: 64 }).notNull().unique(), // Human-readable ID like "TASK-1"
  userId: int("userId").notNull(),
  title: varchar("title", { length: 255 }).notNull(),
  description: text("description"),
  status: mysqlEnum("status", ["backlog", "todo", "in_progress", "review", "done", "cancelled"]).default("todo").notNull(),
  priority: mysqlEnum("priority", ["low", "medium", "high", "urgent"]).default("medium").notNull(),
  assignee: varchar("assignee", { length: 255 }),
  dueDate: timestamp("dueDate"),
  tags: json("tags").$type<string[]>().default([]),
  estimatedTime: int("estimatedTime"), // in minutes
  progress: int("progress").default(0).notNull(), // 0-100
  starred: boolean("starred").default(false).notNull(),
  // Vibe-kanban inspired features
  parentTaskId: int("parentTaskId"), // For hierarchical tasks/subtasks
  sharedTaskId: varchar("sharedTaskId", { length: 128 }), // For shared/collaborative tasks
  projectId: varchar("projectId", { length: 128 }), // Multi-project support
  hasInProgressAttempt: boolean("hasInProgressAttempt").default(false).notNull(),
  lastAttemptFailed: boolean("lastAttemptFailed").default(false).notNull(),
  executor: varchar("executor", { length: 128 }), // AI agent or user
  // Recurring task fields
  isRecurring: boolean("isRecurring").default(false).notNull(),
  recurrencePattern: varchar("recurrencePattern", { length: 128 }), // "daily", "weekly", "monthly", "custom:0 9 * * 1" (cron)
  recurrenceEndDate: timestamp("recurrenceEndDate"),
  parentRecurringTaskId: int("parentRecurringTaskId"), // Reference to the original recurring task
  createdAt: timestamp("createdAt").defaultNow().notNull(),
  updatedAt: timestamp("updatedAt").defaultNow().onUpdateNow().notNull(),
});

export type Task = typeof tasks.$inferSelect;
export type InsertTask = typeof tasks.$inferInsert;

/**
 * Task comments table
 */
export const taskComments = mysqlTable("task_comments", {
  id: int("id").autoincrement().primaryKey(),
  taskId: int("taskId").notNull(),
  author: varchar("author", { length: 255 }).notNull(),
  content: text("content").notNull(),
  avatar: text("avatar"),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
});

export type TaskComment = typeof taskComments.$inferSelect;
export type InsertTaskComment = typeof taskComments.$inferInsert;

/**
 * Task activities table for audit trail
 */
export const taskActivities = mysqlTable("task_activities", {
  id: int("id").autoincrement().primaryKey(),
  taskId: int("taskId").notNull(),
  type: mysqlEnum("type", ["created", "updated", "commented", "status_changed", "assigned"]).notNull(),
  user: varchar("user", { length: 255 }).notNull(),
  description: text("description").notNull(),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
});

export type TaskActivity = typeof taskActivities.$inferSelect;
export type InsertTaskActivity = typeof taskActivities.$inferInsert;

/**
 * Task time entries table for time tracking
 */
export const taskTimeEntries = mysqlTable("task_time_entries", {
  id: int("id").autoincrement().primaryKey(),
  taskId: int("taskId").notNull(),
  duration: int("duration").notNull(), // in minutes
  description: text("description").notNull(),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
});

export type TaskTimeEntry = typeof taskTimeEntries.$inferSelect;
export type InsertTaskTimeEntry = typeof taskTimeEntries.$inferInsert;

/**
 * Task dependencies table
 */
export const taskDependencies = mysqlTable("task_dependencies", {
  id: int("id").autoincrement().primaryKey(),
  taskId: int("taskId").notNull(),
  dependsOnTaskId: int("dependsOnTaskId").notNull(),
  createdAt: timestamp("createdAt").defaultNow().notNull(),
});

export type TaskDependency = typeof taskDependencies.$inferSelect;
export type InsertTaskDependency = typeof taskDependencies.$inferInsert;
