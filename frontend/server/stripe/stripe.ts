import Stripe from "stripe";

// Initialize Stripe with secret key
const stripeSecretKey = process.env.STRIPE_SECRET_KEY;

if (!stripeSecretKey) {
  console.warn("[Stripe] STRIPE_SECRET_KEY not configured. Payment features will be disabled.");
}

export const stripe = stripeSecretKey ? new Stripe(stripeSecretKey) : null;

export interface CreateCheckoutSessionParams {
  userId: number;
  userEmail: string;
  userName?: string;
  priceId: string;
  mode: "subscription" | "payment";
  successUrl: string;
  cancelUrl: string;
  metadata?: Record<string, string>;
}

/**
 * Create a Stripe Checkout Session
 */
export async function createCheckoutSession(params: CreateCheckoutSessionParams): Promise<string | null> {
  if (!stripe) {
    throw new Error("Stripe is not configured");
  }

  const session = await stripe.checkout.sessions.create({
    mode: params.mode,
    payment_method_types: ["card"],
    line_items: [
      {
        price: params.priceId,
        quantity: 1,
      },
    ],
    customer_email: params.userEmail,
    client_reference_id: params.userId.toString(),
    metadata: {
      user_id: params.userId.toString(),
      customer_email: params.userEmail,
      customer_name: params.userName ?? "",
      ...params.metadata,
    },
    success_url: params.successUrl,
    cancel_url: params.cancelUrl,
    allow_promotion_codes: true,
  });

  return session.url;
}

/**
 * Create a Stripe Customer Portal session for managing subscriptions
 */
export async function createCustomerPortalSession(
  customerId: string,
  returnUrl: string
): Promise<string | null> {
  if (!stripe) {
    throw new Error("Stripe is not configured");
  }

  const session = await stripe.billingPortal.sessions.create({
    customer: customerId,
    return_url: returnUrl,
  });

  return session.url;
}

/**
 * Get or create a Stripe customer for a user
 */
export async function getOrCreateCustomer(
  userId: number,
  email: string,
  name?: string
): Promise<string | null> {
  if (!stripe) {
    throw new Error("Stripe is not configured");
  }

  // Search for existing customer by email
  const customers = await stripe.customers.list({
    email,
    limit: 1,
  });

  if (customers.data.length > 0) {
    return customers.data[0].id;
  }

  // Create new customer
  const customer = await stripe.customers.create({
    email,
    name: name ?? undefined,
    metadata: {
      user_id: userId.toString(),
    },
  });

  return customer.id;
}

/**
 * Cancel a subscription
 */
export async function cancelSubscription(subscriptionId: string, cancelAtPeriodEnd = true): Promise<void> {
  if (!stripe) {
    throw new Error("Stripe is not configured");
  }

  if (cancelAtPeriodEnd) {
    await stripe.subscriptions.update(subscriptionId, {
      cancel_at_period_end: true,
    });
  } else {
    await stripe.subscriptions.cancel(subscriptionId);
  }
}

/**
 * Get subscription details
 */
export async function getSubscription(subscriptionId: string): Promise<Stripe.Subscription | null> {
  if (!stripe) {
    return null;
  }

  try {
    return await stripe.subscriptions.retrieve(subscriptionId);
  } catch {
    return null;
  }
}

/**
 * Verify webhook signature
 */
export function constructWebhookEvent(
  payload: Buffer,
  signature: string
): Stripe.Event | null {
  if (!stripe) {
    return null;
  }

  const webhookSecret = process.env.STRIPE_WEBHOOK_SECRET;
  if (!webhookSecret) {
    console.error("[Stripe] STRIPE_WEBHOOK_SECRET not configured");
    return null;
  }

  try {
    return stripe.webhooks.constructEvent(payload, signature, webhookSecret);
  } catch (err) {
    console.error("[Stripe] Webhook signature verification failed:", err);
    return null;
  }
}
