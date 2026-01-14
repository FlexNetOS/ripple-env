import { describe, expect, it, vi, beforeEach } from "vitest";
import { appRouter } from "./routers";
import type { TrpcContext } from "./_core/context";

// Mock the db module
vi.mock("./db", () => ({
  getActivePlans: vi.fn().mockResolvedValue([
    {
      id: 1,
      name: "Free",
      description: "Basic features",
      priceMonthly: 0,
      priceYearly: 0,
      features: { storage: "1GB", ai_messages: 100 },
      isActive: true,
      createdAt: new Date(),
      updatedAt: new Date(),
    },
    {
      id: 2,
      name: "Pro",
      description: "Advanced features",
      priceMonthly: 999,
      priceYearly: 9990,
      features: { storage: "10GB", ai_messages: 1000 },
      isActive: true,
      createdAt: new Date(),
      updatedAt: new Date(),
    },
  ]),
  getSubscriptionByUserId: vi.fn().mockResolvedValue({
    id: 1,
    userId: 1,
    planId: 1,
    stripeCustomerId: "cus_test123",
    stripeSubscriptionId: "sub_test123",
    status: "active",
    billingCycle: "monthly",
    currentPeriodStart: new Date(),
    currentPeriodEnd: new Date(Date.now() + 30 * 24 * 60 * 60 * 1000),
    cancelAtPeriodEnd: false,
    createdAt: new Date(),
    updatedAt: new Date(),
  }),
  getPlanById: vi.fn().mockResolvedValue({
    id: 1,
    name: "Free",
    description: "Basic features",
    priceMonthly: 0,
    priceYearly: 0,
    features: { storage: "1GB", ai_messages: 100 },
    isActive: true,
    createdAt: new Date(),
    updatedAt: new Date(),
  }),
  getPaymentsByUserId: vi.fn().mockResolvedValue([
    {
      id: 1,
      userId: 1,
      stripePaymentIntentId: "pi_test123",
      amount: 999,
      currency: "usd",
      status: "succeeded",
      description: "Pro Plan - Monthly",
      createdAt: new Date(),
    },
  ]),
}));

// Mock stripe module
vi.mock("./stripe/stripe", () => ({
  createCheckoutSession: vi.fn().mockResolvedValue("https://checkout.stripe.com/test"),
  createCustomerPortalSession: vi.fn().mockResolvedValue("https://billing.stripe.com/test"),
  getOrCreateCustomer: vi.fn().mockResolvedValue("cus_test123"),
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
      headers: {
        origin: "http://localhost:3000",
      },
    } as TrpcContext["req"],
    res: {
      clearCookie: vi.fn(),
    } as unknown as TrpcContext["res"],
  };
}

function createUnauthContext(): TrpcContext {
  return {
    user: null,
    req: {
      protocol: "https",
      headers: {},
    } as TrpcContext["req"],
    res: {
      clearCookie: vi.fn(),
    } as unknown as TrpcContext["res"],
  };
}

describe("subscription.getPlans", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("returns list of active plans (public)", async () => {
    const ctx = createUnauthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.subscription.getPlans();

    expect(result).toHaveLength(2);
    expect(result[0].name).toBe("Free");
    expect(result[1].name).toBe("Pro");
  });
});

describe("subscription.getPlansConfig", () => {
  it("returns plans from config with formatted prices", async () => {
    const ctx = createUnauthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.subscription.getPlansConfig();

    expect(result).toBeDefined();
    expect(Array.isArray(result)).toBe(true);
    expect(result.length).toBeGreaterThan(0);
    expect(result[0]).toHaveProperty("priceMonthlyFormatted");
    expect(result[0]).toHaveProperty("priceYearlyFormatted");
  });
});

describe("subscription.getCurrentSubscription", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("returns current subscription for authenticated user", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.subscription.getCurrentSubscription();

    expect(result).toBeDefined();
    expect(result?.status).toBe("active");
    expect(result?.plan?.name).toBe("Free");
  });
});

describe("subscription.getPaymentHistory", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("returns payment history for authenticated user", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    const result = await caller.subscription.getPaymentHistory();

    expect(result).toHaveLength(1);
    expect(result[0].amount).toBe(999);
    expect(result[0].status).toBe("succeeded");
  });
});

describe("subscription.createCheckout", () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it("throws error when plan has no Stripe price ID configured", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    // Pro plan doesn't have stripePriceIdMonthly configured in test environment
    await expect(
      caller.subscription.createCheckout({
        planId: "pro",
        billingCycle: "monthly",
      })
    ).rejects.toThrow("Price not configured for this plan");
  });

  it("throws error for non-existent plan", async () => {
    const ctx = createAuthContext();
    const caller = appRouter.createCaller(ctx);

    await expect(
      caller.subscription.createCheckout({
        planId: "non-existent",
        billingCycle: "monthly",
      })
    ).rejects.toThrow("Plan not found");
  });
});
