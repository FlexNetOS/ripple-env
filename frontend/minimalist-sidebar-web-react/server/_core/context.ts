import type { CreateExpressContextOptions } from "@trpc/server/adapters/express";
import type { User } from "../../drizzle/schema";
import { sdk } from "./sdk";
import { ENV } from "./env";

export type TrpcContext = {
  req: CreateExpressContextOptions["req"];
  res: CreateExpressContextOptions["res"];
  user: User | null;
};

export async function createContext(
  opts: CreateExpressContextOptions
): Promise<TrpcContext> {
  let user: User | null = null;

  // Demo mode: bypass authentication with a mock user
  if (ENV.authMode === 'demo') {
    console.log('[Auth] Demo mode enabled - using mock user');
    user = {
      id: 1,
      openId: 'demo-openid-123',
      name: 'Demo User',
      email: 'demo@ripple.example',
      avatarUrl: null,
      role: 'user',
      loginMethod: 'demo',
      lastSignedIn: new Date(),
      createdAt: new Date(),
      updatedAt: new Date(),
    } as User;
  } else {
    try {
      user = await sdk.authenticateRequest(opts.req);
    } catch (error) {
      // Authentication is optional for public procedures.
      user = null;
    }
  }

  return {
    req: opts.req,
    res: opts.res,
    user,
  };
}
