import { eq, desc, and, sql } from "drizzle-orm";
import { drizzle } from "drizzle-orm/mysql2";
import {
  InsertUser, users,
  InsertFile, files,
  InsertConversation, conversations,
  InsertMessage, messages,
  InsertNotification, notifications,
  InsertSubscriptionPlan, subscriptionPlans,
  InsertSubscription, subscriptions,
  InsertPayment, payments,
  InsertUserSettings, userSettings,
  InsertFavorite, favorites,
  InsertTask, tasks,
  InsertTaskComment, taskComments,
  InsertTaskActivity, taskActivities,
  InsertTaskTimeEntry, taskTimeEntries,
  InsertTaskDependency, taskDependencies,
} from "../drizzle/schema";
import { ENV } from './_core/env';

let _db: ReturnType<typeof drizzle> | null = null;

export async function getDb() {
  if (!_db && process.env.DATABASE_URL) {
    try {
      _db = drizzle(process.env.DATABASE_URL);
    } catch (error) {
      console.warn("[Database] Failed to connect:", error);
      _db = null;
    }
  }
  return _db;
}

// ==================== USER QUERIES ====================

export async function upsertUser(user: InsertUser): Promise<void> {
  if (!user.openId) {
    throw new Error("User openId is required for upsert");
  }

  const db = await getDb();
  if (!db) {
    console.warn("[Database] Cannot upsert user: database not available");
    return;
  }

  try {
    const values: InsertUser = {
      openId: user.openId,
    };
    const updateSet: Record<string, unknown> = {};

    const textFields = ["name", "email", "loginMethod", "avatarUrl"] as const;
    type TextField = (typeof textFields)[number];

    const assignNullable = (field: TextField) => {
      const value = user[field];
      if (value === undefined) return;
      const normalized = value ?? null;
      values[field] = normalized;
      updateSet[field] = normalized;
    };

    textFields.forEach(assignNullable);

    if (user.lastSignedIn !== undefined) {
      values.lastSignedIn = user.lastSignedIn;
      updateSet.lastSignedIn = user.lastSignedIn;
    }
    if (user.role !== undefined) {
      values.role = user.role;
      updateSet.role = user.role;
    } else if (user.openId === ENV.ownerOpenId) {
      values.role = 'admin';
      updateSet.role = 'admin';
    }

    if (!values.lastSignedIn) {
      values.lastSignedIn = new Date();
    }

    if (Object.keys(updateSet).length === 0) {
      updateSet.lastSignedIn = new Date();
    }

    await db.insert(users).values(values).onDuplicateKeyUpdate({
      set: updateSet,
    });
  } catch (error) {
    console.error("[Database] Failed to upsert user:", error);
    throw error;
  }
}

export async function getUserByOpenId(openId: string) {
  const db = await getDb();
  if (!db) {
    console.warn("[Database] Cannot get user: database not available");
    return undefined;
  }

  const result = await db.select().from(users).where(eq(users.openId, openId)).limit(1);
  return result.length > 0 ? result[0] : undefined;
}

export async function getUserById(id: number) {
  const db = await getDb();
  if (!db) return undefined;

  const result = await db.select().from(users).where(eq(users.id, id)).limit(1);
  return result.length > 0 ? result[0] : undefined;
}

export async function updateUserProfile(userId: number, data: { name?: string; email?: string; avatarUrl?: string }) {
  const db = await getDb();
  if (!db) return undefined;

  await db.update(users).set(data).where(eq(users.id, userId));
  return getUserById(userId);
}

// ==================== FILE QUERIES ====================

export async function createFile(file: InsertFile) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(files).values(file);
  return { id: Number(result[0].insertId), ...file };
}

export async function getFilesByUserId(userId: number) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(files).where(eq(files.userId, userId)).orderBy(desc(files.createdAt));
}

export async function getFileById(id: number) {
  const db = await getDb();
  if (!db) return undefined;

  const result = await db.select().from(files).where(eq(files.id, id)).limit(1);
  return result.length > 0 ? result[0] : undefined;
}

export async function deleteFile(id: number, userId: number) {
  const db = await getDb();
  if (!db) return false;

  const result = await db.delete(files).where(and(eq(files.id, id), eq(files.userId, userId)));
  return result[0].affectedRows > 0;
}

// ==================== CONVERSATION QUERIES ====================

export async function createConversation(conv: InsertConversation) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(conversations).values(conv);
  return { id: Number(result[0].insertId), ...conv };
}

export async function getConversationsByUserId(userId: number) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(conversations).where(eq(conversations.userId, userId)).orderBy(desc(conversations.updatedAt));
}

export async function getConversationById(id: number) {
  const db = await getDb();
  if (!db) return undefined;

  const result = await db.select().from(conversations).where(eq(conversations.id, id)).limit(1);
  return result.length > 0 ? result[0] : undefined;
}

export async function updateConversationTitle(id: number, title: string) {
  const db = await getDb();
  if (!db) return;

  await db.update(conversations).set({ title }).where(eq(conversations.id, id));
}

export async function deleteConversation(id: number, userId: number) {
  const db = await getDb();
  if (!db) return false;

  // Delete messages first
  await db.delete(messages).where(eq(messages.conversationId, id));
  const result = await db.delete(conversations).where(and(eq(conversations.id, id), eq(conversations.userId, userId)));
  return result[0].affectedRows > 0;
}

// ==================== MESSAGE QUERIES ====================

export async function createMessage(msg: InsertMessage) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(messages).values(msg);
  
  // Update conversation's updatedAt
  await db.update(conversations).set({ updatedAt: new Date() }).where(eq(conversations.id, msg.conversationId));
  
  return { id: Number(result[0].insertId), ...msg };
}

export async function getMessagesByConversationId(conversationId: number) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(messages).where(eq(messages.conversationId, conversationId)).orderBy(messages.createdAt);
}

// ==================== NOTIFICATION QUERIES ====================

export async function createNotification(notif: InsertNotification) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(notifications).values(notif);
  return { id: Number(result[0].insertId), ...notif };
}

export async function getNotificationsByUserId(userId: number, limit = 50) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(notifications).where(eq(notifications.userId, userId)).orderBy(desc(notifications.createdAt)).limit(limit);
}

export async function getUnreadNotificationCount(userId: number) {
  const db = await getDb();
  if (!db) return 0;

  const result = await db.select({ count: sql<number>`count(*)` }).from(notifications).where(and(eq(notifications.userId, userId), eq(notifications.isRead, false)));
  return result[0]?.count ?? 0;
}

export async function markNotificationAsRead(id: number, userId: number) {
  const db = await getDb();
  if (!db) return false;

  const result = await db.update(notifications).set({ isRead: true }).where(and(eq(notifications.id, id), eq(notifications.userId, userId)));
  return result[0].affectedRows > 0;
}

export async function markAllNotificationsAsRead(userId: number) {
  const db = await getDb();
  if (!db) return;

  await db.update(notifications).set({ isRead: true }).where(eq(notifications.userId, userId));
}

export async function deleteNotification(id: number, userId: number) {
  const db = await getDb();
  if (!db) return false;

  const result = await db.delete(notifications).where(and(eq(notifications.id, id), eq(notifications.userId, userId)));
  return result[0].affectedRows > 0;
}

// ==================== SUBSCRIPTION PLAN QUERIES ====================

export async function getActivePlans() {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(subscriptionPlans).where(eq(subscriptionPlans.isActive, true));
}

export async function getPlanById(id: number) {
  const db = await getDb();
  if (!db) return undefined;

  const result = await db.select().from(subscriptionPlans).where(eq(subscriptionPlans.id, id)).limit(1);
  return result.length > 0 ? result[0] : undefined;
}

export async function createPlan(plan: InsertSubscriptionPlan) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(subscriptionPlans).values(plan);
  return { id: Number(result[0].insertId), ...plan };
}

// ==================== SUBSCRIPTION QUERIES ====================

export async function getSubscriptionByUserId(userId: number) {
  const db = await getDb();
  if (!db) return undefined;

  const result = await db.select().from(subscriptions).where(eq(subscriptions.userId, userId)).limit(1);
  return result.length > 0 ? result[0] : undefined;
}

export async function createSubscription(sub: InsertSubscription) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(subscriptions).values(sub);
  return { id: Number(result[0].insertId), ...sub };
}

export async function updateSubscription(id: number, data: Partial<InsertSubscription>) {
  const db = await getDb();
  if (!db) return;

  await db.update(subscriptions).set(data).where(eq(subscriptions.id, id));
}

export async function getSubscriptionByStripeId(stripeSubscriptionId: string) {
  const db = await getDb();
  if (!db) return undefined;

  const result = await db.select().from(subscriptions).where(eq(subscriptions.stripeSubscriptionId, stripeSubscriptionId)).limit(1);
  return result.length > 0 ? result[0] : undefined;
}

// ==================== PAYMENT QUERIES ====================

export async function createPayment(payment: InsertPayment) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(payments).values(payment);
  return { id: Number(result[0].insertId), ...payment };
}

export async function getPaymentsByUserId(userId: number) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(payments).where(eq(payments.userId, userId)).orderBy(desc(payments.createdAt));
}

// ==================== USER SETTINGS QUERIES ====================

export async function getUserSettings(userId: number) {
  const db = await getDb();
  if (!db) return undefined;

  const result = await db.select().from(userSettings).where(eq(userSettings.userId, userId)).limit(1);
  return result.length > 0 ? result[0] : undefined;
}

export async function upsertUserSettings(userId: number, settings: Partial<InsertUserSettings>) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const existing = await getUserSettings(userId);
  if (existing) {
    await db.update(userSettings).set(settings).where(eq(userSettings.userId, userId));
  } else {
    await db.insert(userSettings).values({ userId, ...settings });
  }
  return getUserSettings(userId);
}


// ==================== FAVORITES QUERIES ====================

export async function getFavoritesByUserId(userId: number) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(favorites).where(eq(favorites.userId, userId)).orderBy(favorites.sortOrder);
}

export async function addFavorite(favorite: InsertFavorite) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  // Check if already exists
  const existing = await db.select().from(favorites).where(
    and(
      eq(favorites.userId, favorite.userId),
      eq(favorites.itemId, favorite.itemId)
    )
  ).limit(1);

  if (existing.length > 0) {
    return existing[0];
  }

  // Get max sort order
  const maxOrder = await db.select({ maxOrder: sql<number>`COALESCE(MAX(${favorites.sortOrder}), 0)` })
    .from(favorites)
    .where(eq(favorites.userId, favorite.userId));
  
  const newOrder = (maxOrder[0]?.maxOrder ?? 0) + 1;

  const result = await db.insert(favorites).values({ ...favorite, sortOrder: newOrder });
  return { id: Number(result[0].insertId), ...favorite, sortOrder: newOrder };
}

export async function removeFavorite(userId: number, itemId: string) {
  const db = await getDb();
  if (!db) return false;

  const result = await db.delete(favorites).where(
    and(eq(favorites.userId, userId), eq(favorites.itemId, itemId))
  );
  return result[0].affectedRows > 0;
}

export async function reorderFavorites(userId: number, itemIds: string[]) {
  const db = await getDb();
  if (!db) return;

  for (let i = 0; i < itemIds.length; i++) {
    await db.update(favorites)
      .set({ sortOrder: i })
      .where(and(eq(favorites.userId, userId), eq(favorites.itemId, itemIds[i])));
  }
}

export async function isFavorite(userId: number, itemId: string) {
  const db = await getDb();
  if (!db) return false;

  const result = await db.select().from(favorites).where(
    and(eq(favorites.userId, userId), eq(favorites.itemId, itemId))
  ).limit(1);
  return result.length > 0;
}

// ==================== TASK QUERIES ====================

export async function createTask(task: InsertTask) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(tasks).values(task);
  return { id: Number(result[0].insertId), ...task };
}

export async function getTasksByUserId(userId: number) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(tasks).where(eq(tasks.userId, userId)).orderBy(desc(tasks.createdAt));
}

export async function getTaskById(id: number) {
  const db = await getDb();
  if (!db) return undefined;

  const result = await db.select().from(tasks).where(eq(tasks.id, id)).limit(1);
  return result.length > 0 ? result[0] : undefined;
}

export async function updateTask(id: number, userId: number, data: Partial<InsertTask>) {
  const db = await getDb();
  if (!db) return undefined;

  await db.update(tasks).set({ ...data, updatedAt: new Date() }).where(and(eq(tasks.id, id), eq(tasks.userId, userId)));
  return getTaskById(id);
}

export async function deleteTask(id: number, userId: number) {
  const db = await getDb();
  if (!db) return false;

  // Delete related records first
  await db.delete(taskComments).where(eq(taskComments.taskId, id));
  await db.delete(taskActivities).where(eq(taskActivities.taskId, id));
  await db.delete(taskTimeEntries).where(eq(taskTimeEntries.taskId, id));
  await db.delete(taskDependencies).where(eq(taskDependencies.taskId, id));
  
  const result = await db.delete(tasks).where(and(eq(tasks.id, id), eq(tasks.userId, userId)));
  return result[0].affectedRows > 0;
}

export async function getSubtasks(parentTaskId: number) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(tasks).where(eq(tasks.parentTaskId, parentTaskId)).orderBy(tasks.createdAt);
}

// ==================== TASK COMMENT QUERIES ====================

export async function createTaskComment(comment: InsertTaskComment) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(taskComments).values(comment);
  return { id: Number(result[0].insertId), ...comment };
}

export async function getTaskComments(taskId: number) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(taskComments).where(eq(taskComments.taskId, taskId)).orderBy(taskComments.createdAt);
}

export async function deleteTaskComment(id: number) {
  const db = await getDb();
  if (!db) return false;

  const result = await db.delete(taskComments).where(eq(taskComments.id, id));
  return result[0].affectedRows > 0;
}

// ==================== TASK ACTIVITY QUERIES ====================

export async function createTaskActivity(activity: InsertTaskActivity) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(taskActivities).values(activity);
  return { id: Number(result[0].insertId), ...activity };
}

export async function getTaskActivities(taskId: number) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(taskActivities).where(eq(taskActivities.taskId, taskId)).orderBy(desc(taskActivities.createdAt));
}

// ==================== TASK TIME ENTRY QUERIES ====================

export async function createTaskTimeEntry(entry: InsertTaskTimeEntry) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(taskTimeEntries).values(entry);
  return { id: Number(result[0].insertId), ...entry };
}

export async function getTaskTimeEntries(taskId: number) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(taskTimeEntries).where(eq(taskTimeEntries.taskId, taskId)).orderBy(taskTimeEntries.createdAt);
}

export async function deleteTaskTimeEntry(id: number) {
  const db = await getDb();
  if (!db) return false;

  const result = await db.delete(taskTimeEntries).where(eq(taskTimeEntries.id, id));
  return result[0].affectedRows > 0;
}

// ==================== TASK DEPENDENCY QUERIES ====================

export async function createTaskDependency(dependency: InsertTaskDependency) {
  const db = await getDb();
  if (!db) throw new Error("Database not available");

  const result = await db.insert(taskDependencies).values(dependency);
  return { id: Number(result[0].insertId), ...dependency };
}

export async function getTaskDependencies(taskId: number) {
  const db = await getDb();
  if (!db) return [];

  return db.select().from(taskDependencies).where(eq(taskDependencies.taskId, taskId));
}

export async function deleteTaskDependency(id: number) {
  const db = await getDb();
  if (!db) return false;

  const result = await db.delete(taskDependencies).where(eq(taskDependencies.id, id));
  return result[0].affectedRows > 0;
}
