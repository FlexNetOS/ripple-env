import { describe, expect, it, vi, beforeEach } from "vitest";
import { appRouter } from "./routers";
import type { TrpcContext } from "./_core/context";

// Mock the db module
vi.mock("./db", () => ({
  updateUserProfile: vi.fn().mockResolvedValue({
    id: 1,
    openId: "test-user",
    name: "Updated Name",
    email: "updated@example.com",
    loginMethod: "manus",
    role: "user",
    createdAt: new Date(),
    updatedAt: new Date(),
    lastSignedIn: new Date(),
  }),
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
    emailNotifications: true,
    pushNotifications: false,
    theme: "dark",
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

describe("user.getProfile", () => {
  it("returns the authenticated user profile", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.user.getProfile();

    expect(result).toEqual(ctx.user);
    expect(result?.id).toBe(1);
    expect(result?.email).toBe("test@example.com");
  });
});

describe("user.updateProfile", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("updates user profile with name and email", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.user.updateProfile({
      name: "Updated Name",
      email: "updated@example.com",
    });

    expect(result).toBeDefined();
    expect(result?.name).toBe("Updated Name");
    expect(result?.email).toBe("updated@example.com");
  });

  it("updates only name when email is not provided", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.user.updateProfile({
      name: "New Name Only",
    });

    expect(result).toBeDefined();
  });
});

describe("user.getSettings", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("returns user settings", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.user.getSettings();

    expect(result).toBeDefined();
    expect(result?.emailNotifications).toBe(true);
    expect(result?.pushNotifications).toBe(true);
  });
});

describe("user.updateSettings", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("updates user settings", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.user.updateSettings({
      emailNotifications: true,
      pushNotifications: false,
      theme: "dark",
    });

    expect(result).toBeDefined();
  });
});
