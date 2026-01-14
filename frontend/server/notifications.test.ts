import { describe, expect, it, vi, beforeEach } from "vitest";
import { appRouter } from "./routers";
import type { TrpcContext } from "./_core/context";

// Mock the db module
vi.mock("./db", () => ({
  getNotificationsByUserId: vi.fn().mockResolvedValue([
    {
      id: 1,
      userId: 1,
      title: "Welcome",
      content: "Welcome to the platform!",
      type: "info",
      isRead: false,
      link: null,
      createdAt: new Date(),
    },
    {
      id: 2,
      userId: 1,
      title: "Payment Received",
      content: "Your payment was successful.",
      type: "success",
      isRead: true,
      link: "/billing",
      createdAt: new Date(),
    },
  ]),
  getUnreadNotificationCount: vi.fn().mockResolvedValue(1),
  markNotificationAsRead: vi.fn().mockResolvedValue(true),
  markAllNotificationsAsRead: vi.fn().mockResolvedValue(undefined),
  getUserSettings: vi.fn().mockResolvedValue({
    id: 1,
    userId: 1,
    emailNotifications: true,
    pushNotifications: true,
    theme: "light",
    language: "en",
    createdAt: new Date(),
    updatedAt: new Date(),
  }),
  upsertUserSettings: vi.fn().mockResolvedValue({
    id: 1,
    userId: 1,
    emailNotifications: false,
    pushNotifications: true,
    theme: "light",
    language: "en",
    createdAt: new Date(),
    updatedAt: new Date(),
  }),
}));

type AuthenticatedUser = NonNullable<TrpcContext["user"]>;

function createAuthContext(): TrpcContext {
  const user: AuthenticatedUser = {
    id: 1,
    openId: "test-user",
    email: "test@example.com",
    name: "Test User",
    loginMethod: "manus",
    role: "user",
    createdAt: new Date(),
    updatedAt: new Date(),
    lastSignedIn: new Date(),
  };

  return {
    user,
    req: {
      protocol: "https",
      headers: {},
    } as TrpcContext["req"],
    res: {
      clearCookie: vi.fn(),
    } as unknown as TrpcContext["res"],
  };
}

describe("notifications.list", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("returns list of notifications for authenticated user", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.notifications.list({});

    expect(result).toHaveLength(2);
    expect(result[0].title).toBe("Welcome");
    expect(result[1].title).toBe("Payment Received");
  });

  it("respects limit parameter", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.notifications.list({ limit: 1 });

    expect(result).toBeDefined();
  });
});

describe("notifications.unreadCount", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("returns count of unread notifications", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.notifications.unreadCount();

    expect(result).toBe(1);
  });
});

describe("notifications.markAsRead", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("marks a notification as read", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.notifications.markAsRead({ id: 1 });

    expect(result).toEqual({ success: true });
  });
});

describe("notifications.markAllAsRead", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("marks all notifications as read", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.notifications.markAllAsRead();

    expect(result).toEqual({ success: true });
  });
});

describe("notifications.getPreferences", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("returns notification preferences", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.notifications.getPreferences();

    expect(result).toEqual({
      emailEnabled: true,
      pushEnabled: true,
    });
  });
});

describe("notifications.updatePreferences", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("updates notification preferences", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.notifications.updatePreferences({
      emailEnabled: false,
      pushEnabled: true,
    });

    expect(result).toEqual({ success: true });
  });
});
