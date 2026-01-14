import { COOKIE_NAME } from "@shared/const";
import { getSessionCookieOptions } from "./_core/cookies";
import { systemRouter } from "./_core/systemRouter";
import { publicProcedure, protectedProcedure, router } from "./_core/trpc";
import { TRPCError } from "@trpc/server";
import { z } from "zod";
import { invokeLLM } from "./_core/llm";
import { storagePut } from "./storage";
import { nanoid } from "nanoid";
import * as db from "./db";
import { createCheckoutSession, createCustomerPortalSession, getOrCreateCustomer } from "./stripe/stripe";
import { SUBSCRIPTION_PLANS, formatPrice } from "./stripe/products";

// Admin-only procedure
const adminProcedure = protectedProcedure.use(({ ctx, next }) => {
  if (ctx.user.role !== "admin") {
    throw new TRPCError({ code: "FORBIDDEN", message: "Admin access required" });
  }
  return next({ ctx });
});

export const appRouter = router({
  system: systemRouter,

  // ==================== AUTH ROUTER ====================
  auth: router({
    me: publicProcedure.query((opts) => opts.ctx.user),
    logout: publicProcedure.mutation(({ ctx }) => {
      const cookieOptions = getSessionCookieOptions(ctx.req);
      ctx.res.clearCookie(COOKIE_NAME, { ...cookieOptions, maxAge: -1 });
      return { success: true } as const;
    }),
  }),

  // ==================== USER ROUTER ====================
  user: router({
    getProfile: protectedProcedure.query(async ({ ctx }) => {
      return ctx.user;
    }),

    updateProfile: protectedProcedure
      .input(z.object({
        name: z.string().min(1).max(100).optional(),
        email: z.string().email().optional(),
      }))
      .mutation(async ({ ctx, input }) => {
        const updated = await db.updateUserProfile(ctx.user.id, input);
        return updated;
      }),

    getSettings: protectedProcedure.query(async ({ ctx }) => {
      let settings = await db.getUserSettings(ctx.user.id);
      if (!settings) {
        settings = await db.upsertUserSettings(ctx.user.id, {});
      }
      return settings;
    }),

    updateSettings: protectedProcedure
      .input(z.object({
        emailNotifications: z.boolean().optional(),
        pushNotifications: z.boolean().optional(),
        theme: z.enum(["light", "dark", "system"]).optional(),
        language: z.string().max(10).optional(),
      }))
      .mutation(async ({ ctx, input }) => {
        return db.upsertUserSettings(ctx.user.id, input);
      }),

    uploadAvatar: protectedProcedure
      .input(z.object({
        base64Data: z.string(),
        fileName: z.string().min(1).max(255),
        mimeType: z.string().min(1).max(128),
      }))
      .mutation(async ({ ctx, input }) => {
        // Extract base64 data (remove data URL prefix if present)
        const base64Match = input.base64Data.match(/^data:[^;]+;base64,(.+)$/);
        const base64Content = base64Match ? base64Match[1] : input.base64Data;
        const buffer = Buffer.from(base64Content, 'base64');

        // Generate unique file key for avatar
        const ext = input.fileName.split('.').pop() || 'jpg';
        const fileKey = `avatars/${ctx.user.id}-${nanoid()}.${ext}`;

        // Upload to S3
        const { url } = await storagePut(fileKey, buffer, input.mimeType);

        // Update user profile with new avatar URL
        await db.updateUserProfile(ctx.user.id, { avatarUrl: url });

        return { avatarUrl: url };
      }),
  }),

  // ==================== FILES ROUTER ====================
  files: router({
    list: protectedProcedure.query(async ({ ctx }) => {
      return db.getFilesByUserId(ctx.user.id);
    }),

    upload: protectedProcedure
      .input(z.object({
        fileName: z.string().min(1).max(255),
        fileData: z.string(), // base64 encoded
        mimeType: z.string().optional(),
      }))
      .mutation(async ({ ctx, input }) => {
        const buffer = Buffer.from(input.fileData, "base64");
        const fileKey = `${ctx.user.id}-files/${nanoid()}-${input.fileName}`;
        
        const { url } = await storagePut(fileKey, buffer, input.mimeType);
        
        const file = await db.createFile({
          userId: ctx.user.id,
          fileName: input.fileName,
          fileKey,
          url,
          mimeType: input.mimeType ?? null,
          fileSize: buffer.length,
        });
        
        return file;
      }),

    delete: protectedProcedure
      .input(z.object({ id: z.number() }))
      .mutation(async ({ ctx, input }) => {
        const success = await db.deleteFile(input.id, ctx.user.id);
        if (!success) {
          throw new TRPCError({ code: "NOT_FOUND", message: "File not found" });
        }
        return { success: true };
      }),

    get: protectedProcedure
      .input(z.object({ id: z.number() }))
      .query(async ({ ctx, input }) => {
        const file = await db.getFileById(input.id);
        if (!file || file.userId !== ctx.user.id) {
          throw new TRPCError({ code: "NOT_FOUND", message: "File not found" });
        }
        return file;
      }),
  }),

  // ==================== CHAT ROUTER ====================
  chat: router({
    listConversations: protectedProcedure.query(async ({ ctx }) => {
      return db.getConversationsByUserId(ctx.user.id);
    }),

    createConversation: protectedProcedure
      .input(z.object({ title: z.string().max(255).optional() }))
      .mutation(async ({ ctx, input }) => {
        return db.createConversation({
          userId: ctx.user.id,
          title: input.title ?? "New Conversation",
        });
      }),

    getConversation: protectedProcedure
      .input(z.object({ id: z.number() }))
      .query(async ({ ctx, input }) => {
        const conv = await db.getConversationById(input.id);
        if (!conv || conv.userId !== ctx.user.id) {
          throw new TRPCError({ code: "NOT_FOUND", message: "Conversation not found" });
        }
        const msgs = await db.getMessagesByConversationId(input.id);
        return { ...conv, messages: msgs };
      }),

    deleteConversation: protectedProcedure
      .input(z.object({ id: z.number() }))
      .mutation(async ({ ctx, input }) => {
        const success = await db.deleteConversation(input.id, ctx.user.id);
        if (!success) {
          throw new TRPCError({ code: "NOT_FOUND", message: "Conversation not found" });
        }
        return { success: true };
      }),

    updateTitle: protectedProcedure
      .input(z.object({ id: z.number(), title: z.string().min(1).max(255) }))
      .mutation(async ({ ctx, input }) => {
        const conv = await db.getConversationById(input.id);
        if (!conv || conv.userId !== ctx.user.id) {
          throw new TRPCError({ code: "NOT_FOUND", message: "Conversation not found" });
        }
        await db.updateConversationTitle(input.id, input.title);
        return { success: true };
      }),

    sendMessage: protectedProcedure
      .input(z.object({
        conversationId: z.number(),
        content: z.string().min(1).max(10000),
      }))
      .mutation(async ({ ctx, input }) => {
        const conv = await db.getConversationById(input.conversationId);
        if (!conv || conv.userId !== ctx.user.id) {
          throw new TRPCError({ code: "NOT_FOUND", message: "Conversation not found" });
        }

        // Save user message
        const userMsg = await db.createMessage({
          conversationId: input.conversationId,
          role: "user",
          content: input.content,
        });

        // Get conversation history for context
        const history = await db.getMessagesByConversationId(input.conversationId);
        const messagesForLLM = history.map((m) => ({
          role: m.role as "user" | "assistant" | "system",
          content: m.content,
        }));

        // Generate AI response
        try {
          const response = await invokeLLM({
            messages: [
              {
                role: "system",
                content: "You are a helpful, friendly, and knowledgeable AI assistant. Provide clear, accurate, and helpful responses. Be conversational but professional.",
              },
              ...messagesForLLM,
            ],
          });

          const rawContent = response.choices[0]?.message?.content;
          const assistantContent = typeof rawContent === "string" ? rawContent : "I apologize, but I couldn't generate a response.";

          // Save assistant message
          const assistantMsg = await db.createMessage({
            conversationId: input.conversationId,
            role: "assistant",
            content: assistantContent,
          });

          // Update conversation title if it's the first message
          if (history.length === 0) {
            const titleResponse = await invokeLLM({
              messages: [
                {
                  role: "system",
                  content: "Generate a very short title (3-6 words) for this conversation based on the user's message. Only respond with the title, nothing else.",
                },
                { role: "user", content: input.content },
              ],
            });
            const rawTitle = titleResponse.choices[0]?.message?.content;
            const title = (typeof rawTitle === "string" ? rawTitle : "New Conversation").slice(0, 100);
            await db.updateConversationTitle(input.conversationId, title);
          }

          return { userMessage: userMsg, assistantMessage: assistantMsg };
        } catch (error) {
          console.error("LLM error:", error);
          const errorMsg = await db.createMessage({
            conversationId: input.conversationId,
            role: "assistant",
            content: "I apologize, but I encountered an error processing your request. Please try again.",
          });
          return { userMessage: userMsg, assistantMessage: errorMsg };
        }
      }),
  }),

  // ==================== NOTIFICATIONS ROUTER ====================
  notifications: router({
    list: protectedProcedure
      .input(z.object({ limit: z.number().min(1).max(100).optional() }))
      .query(async ({ ctx, input }) => {
        return db.getNotificationsByUserId(ctx.user.id, input.limit ?? 50);
      }),

    unreadCount: protectedProcedure.query(async ({ ctx }) => {
      return db.getUnreadNotificationCount(ctx.user.id);
    }),

    markAsRead: protectedProcedure
      .input(z.object({ id: z.number() }))
      .mutation(async ({ ctx, input }) => {
        const success = await db.markNotificationAsRead(input.id, ctx.user.id);
        if (!success) {
          throw new TRPCError({ code: "NOT_FOUND", message: "Notification not found" });
        }
        return { success: true };
      }),

    markAllAsRead: protectedProcedure.mutation(async ({ ctx }) => {
      await db.markAllNotificationsAsRead(ctx.user.id);
      return { success: true };
    }),

    // Get notification preferences
    getPreferences: protectedProcedure.query(async ({ ctx }) => {
      const settings = await db.getUserSettings(ctx.user.id);
      if (!settings) {
        const newSettings = await db.upsertUserSettings(ctx.user.id, {});
        return {
          emailEnabled: newSettings?.emailNotifications ?? true,
          pushEnabled: newSettings?.pushNotifications ?? true,
        };
      }
      return {
        emailEnabled: settings.emailNotifications,
        pushEnabled: settings.pushNotifications,
      };
    }),

    // Update notification preferences
    updatePreferences: protectedProcedure
      .input(z.object({
        emailEnabled: z.boolean().optional(),
        pushEnabled: z.boolean().optional(),
      }))
      .mutation(async ({ ctx, input }) => {
        const updateData: { emailNotifications?: boolean; pushNotifications?: boolean } = {};
        if (input.emailEnabled !== undefined) {
          updateData.emailNotifications = input.emailEnabled;
        }
        if (input.pushEnabled !== undefined) {
          updateData.pushNotifications = input.pushEnabled;
        }
        await db.upsertUserSettings(ctx.user.id, updateData);
        return { success: true };
      }),

    // Delete a notification
    delete: protectedProcedure
      .input(z.object({ id: z.number() }))
      .mutation(async ({ ctx, input }) => {
        const success = await db.deleteNotification(input.id, ctx.user.id);
        if (!success) {
          throw new TRPCError({ code: "NOT_FOUND", message: "Notification not found" });
        }
        return { success: true };
      }),

    // Admin: create notification for a user
    create: adminProcedure
      .input(z.object({
        userId: z.number(),
        title: z.string().min(1).max(255),
        content: z.string().min(1),
        type: z.enum(["info", "success", "warning", "error"]).optional(),
        link: z.string().max(512).optional(),
      }))
      .mutation(async ({ input }) => {
        return db.createNotification({
          userId: input.userId,
          title: input.title,
          content: input.content,
          type: input.type ?? "info",
          link: input.link ?? null,
        });
      }),
  }),

  // ==================== FAVORITES ROUTER ====================
  favorites: router({
    list: protectedProcedure.query(async ({ ctx }) => {
      return db.getFavoritesByUserId(ctx.user.id);
    }),

    add: protectedProcedure
      .input(z.object({
        itemId: z.string().min(1).max(128),
        itemType: z.string().min(1).max(64),
        title: z.string().min(1).max(255),
        icon: z.string().max(64).optional(),
        route: z.string().max(255).optional(),
        panelId: z.string().max(128).optional(),
      }))
      .mutation(async ({ ctx, input }) => {
        return db.addFavorite({
          userId: ctx.user.id,
          itemId: input.itemId,
          itemType: input.itemType,
          title: input.title,
          icon: input.icon ?? null,
          route: input.route ?? null,
          panelId: input.panelId ?? null,
        });
      }),

    remove: protectedProcedure
      .input(z.object({ itemId: z.string() }))
      .mutation(async ({ ctx, input }) => {
        const success = await db.removeFavorite(ctx.user.id, input.itemId);
        if (!success) {
          throw new TRPCError({ code: "NOT_FOUND", message: "Favorite not found" });
        }
        return { success: true };
      }),

    reorder: protectedProcedure
      .input(z.object({ itemIds: z.array(z.string()) }))
      .mutation(async ({ ctx, input }) => {
        await db.reorderFavorites(ctx.user.id, input.itemIds);
        return { success: true };
      }),

    isFavorite: protectedProcedure
      .input(z.object({ itemId: z.string() }))
      .query(async ({ ctx, input }) => {
        return db.isFavorite(ctx.user.id, input.itemId);
      }),
  }),

  // ==================== SUBSCRIPTION ROUTER ====================
  subscription: router({
    getPlans: publicProcedure.query(async () => {
      return db.getActivePlans();
    }),

    getCurrentSubscription: protectedProcedure.query(async ({ ctx }) => {
      const sub = await db.getSubscriptionByUserId(ctx.user.id);
      if (!sub) return null;
      
      const plan = await db.getPlanById(sub.planId);
      return { ...sub, plan };
    }),

    getPaymentHistory: protectedProcedure.query(async ({ ctx }) => {
      return db.getPaymentsByUserId(ctx.user.id);
    }),

    // Get plans from config
    getPlansConfig: publicProcedure.query(() => {
      return SUBSCRIPTION_PLANS.map((plan) => ({
        ...plan,
        priceMonthlyFormatted: formatPrice(plan.priceMonthly),
        priceYearlyFormatted: formatPrice(plan.priceYearly),
      }));
    }),

    // Create checkout session
    createCheckout: protectedProcedure
      .input(z.object({
        planId: z.string(),
        billingCycle: z.enum(["monthly", "yearly"]),
      }))
      .mutation(async ({ ctx, input }) => {
        const plan = SUBSCRIPTION_PLANS.find((p) => p.id === input.planId);
        if (!plan) {
          throw new TRPCError({ code: "NOT_FOUND", message: "Plan not found" });
        }

        const priceId = input.billingCycle === "monthly"
          ? plan.stripePriceIdMonthly
          : plan.stripePriceIdYearly;

        if (!priceId) {
          throw new TRPCError({ code: "BAD_REQUEST", message: "Price not configured for this plan" });
        }

        const origin = ctx.req.headers.origin || "http://localhost:3000";

        const checkoutUrl = await createCheckoutSession({
          userId: ctx.user.id,
          userEmail: ctx.user.email ?? "",
          userName: ctx.user.name ?? undefined,
          priceId,
          mode: "subscription",
          successUrl: `${origin}/billing?success=true`,
          cancelUrl: `${origin}/billing?canceled=true`,
          metadata: {
            plan_id: plan.id,
            billing_cycle: input.billingCycle,
          },
        });

        return { url: checkoutUrl };
      }),

    // Create customer portal session
    createPortalSession: protectedProcedure.mutation(async ({ ctx }) => {
      const subscription = await db.getSubscriptionByUserId(ctx.user.id);
      if (!subscription?.stripeCustomerId) {
        throw new TRPCError({ code: "NOT_FOUND", message: "No active subscription found" });
      }

      const origin = ctx.req.headers.origin || "http://localhost:3000";
      const portalUrl = await createCustomerPortalSession(
        subscription.stripeCustomerId,
        `${origin}/billing`
      );

      return { url: portalUrl };
    }),

    // Admin: create a plan
    createPlan: adminProcedure
      .input(z.object({
        name: z.string().min(1).max(64),
        description: z.string().optional(),
        priceMonthly: z.number().min(0),
        priceYearly: z.number().min(0),
        features: z.array(z.string()).optional(),
        stripePriceIdMonthly: z.string().optional(),
        stripePriceIdYearly: z.string().optional(),
      }))
      .mutation(async ({ input }) => {
        return db.createPlan({
          name: input.name,
          description: input.description ?? null,
          priceMonthly: input.priceMonthly,
          priceYearly: input.priceYearly,
          features: input.features ?? [],
          stripePriceIdMonthly: input.stripePriceIdMonthly ?? null,
          stripePriceIdYearly: input.stripePriceIdYearly ?? null,
        });
      }),
  }),
});

export type AppRouter = typeof appRouter;
