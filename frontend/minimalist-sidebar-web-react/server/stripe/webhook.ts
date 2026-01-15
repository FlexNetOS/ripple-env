import { Router, raw } from "express";
import { constructWebhookEvent } from "./stripe";
import * as db from "../db";

export const stripeWebhookRouter = Router();

// Use raw body parser for webhook signature verification
stripeWebhookRouter.post(
  "/webhook",
  raw({ type: "application/json" }),
  async (req, res) => {
    const signature = req.headers["stripe-signature"] as string;

    if (!signature) {
      console.error("[Stripe Webhook] Missing stripe-signature header");
      return res.status(400).json({ error: "Missing signature" });
    }

    const event = constructWebhookEvent(req.body, signature);

    if (!event) {
      console.error("[Stripe Webhook] Invalid signature or webhook secret");
      return res.status(400).json({ error: "Invalid signature" });
    }

    // Handle test events for webhook verification
    if (event.id.startsWith("evt_test_")) {
      console.log("[Stripe Webhook] Test event detected, returning verification response");
      return res.json({ verified: true });
    }

    console.log(`[Stripe Webhook] Received event: ${event.type} (${event.id})`);

    try {
      switch (event.type) {
        case "checkout.session.completed": {
          const session = event.data.object;
          await handleCheckoutCompleted(session);
          break;
        }

        case "customer.subscription.created":
        case "customer.subscription.updated": {
          const subscription = event.data.object;
          await handleSubscriptionUpdate(subscription);
          break;
        }

        case "customer.subscription.deleted": {
          const subscription = event.data.object;
          await handleSubscriptionDeleted(subscription);
          break;
        }

        case "invoice.paid": {
          const invoice = event.data.object;
          await handleInvoicePaid(invoice);
          break;
        }

        case "invoice.payment_failed": {
          const invoice = event.data.object;
          await handleInvoicePaymentFailed(invoice);
          break;
        }

        default:
          console.log(`[Stripe Webhook] Unhandled event type: ${event.type}`);
      }

      res.json({ received: true });
    } catch (error) {
      console.error(`[Stripe Webhook] Error handling ${event.type}:`, error);
      res.status(500).json({ error: "Webhook handler failed" });
    }
  }
);

async function handleCheckoutCompleted(session: any) {
  const userId = session.metadata?.user_id;
  if (!userId) {
    console.error("[Stripe Webhook] No user_id in checkout session metadata");
    return;
  }

  console.log(`[Stripe Webhook] Checkout completed for user ${userId}`);

  // If subscription mode, the subscription webhook will handle the rest
  if (session.mode === "subscription" && session.subscription) {
    console.log(`[Stripe Webhook] Subscription ${session.subscription} created for user ${userId}`);
  }

  // Create a notification for the user
  await db.createNotification({
    userId: parseInt(userId),
    title: "Payment Successful",
    content: "Your payment has been processed successfully. Thank you for your purchase!",
    type: "success",
    link: "/billing",
  });
}

async function handleSubscriptionUpdate(subscription: any) {
  const existingSub = await db.getSubscriptionByStripeId(subscription.id);

  if (existingSub) {
    // Update existing subscription
    await db.updateSubscription(existingSub.id, {
      status: mapStripeStatus(subscription.status),
      currentPeriodStart: new Date(subscription.current_period_start * 1000),
      currentPeriodEnd: new Date(subscription.current_period_end * 1000),
      cancelAtPeriodEnd: subscription.cancel_at_period_end,
    });
    console.log(`[Stripe Webhook] Updated subscription ${subscription.id}`);
  } else {
    // This is a new subscription - we need the user_id from metadata
    const customerId = subscription.customer as string;
    console.log(`[Stripe Webhook] New subscription ${subscription.id} for customer ${customerId}`);
    // The subscription will be linked when we have the user context
  }
}

async function handleSubscriptionDeleted(subscription: any) {
  const existingSub = await db.getSubscriptionByStripeId(subscription.id);

  if (existingSub) {
    await db.updateSubscription(existingSub.id, {
      status: "canceled",
    });

    // Notify user
    await db.createNotification({
      userId: existingSub.userId,
      title: "Subscription Canceled",
      content: "Your subscription has been canceled. You can resubscribe at any time.",
      type: "info",
      link: "/billing",
    });

    console.log(`[Stripe Webhook] Subscription ${subscription.id} canceled`);
  }
}

async function handleInvoicePaid(invoice: any) {
  const subscriptionId = invoice.subscription as string;
  if (!subscriptionId) return;

  const existingSub = await db.getSubscriptionByStripeId(subscriptionId);
  if (!existingSub) return;

  // Record payment
  await db.createPayment({
    userId: existingSub.userId,
    subscriptionId: existingSub.id,
    stripePaymentIntentId: invoice.payment_intent as string,
    amount: invoice.amount_paid,
    currency: invoice.currency,
    status: "succeeded",
    description: `Subscription payment - ${invoice.number}`,
  });

  console.log(`[Stripe Webhook] Invoice ${invoice.id} paid for subscription ${subscriptionId}`);
}

async function handleInvoicePaymentFailed(invoice: any) {
  const subscriptionId = invoice.subscription as string;
  if (!subscriptionId) return;

  const existingSub = await db.getSubscriptionByStripeId(subscriptionId);
  if (!existingSub) return;

  // Update subscription status
  await db.updateSubscription(existingSub.id, {
    status: "past_due",
  });

  // Notify user
  await db.createNotification({
    userId: existingSub.userId,
    title: "Payment Failed",
    content: "Your subscription payment failed. Please update your payment method to avoid service interruption.",
    type: "error",
    link: "/billing",
  });

  console.log(`[Stripe Webhook] Invoice ${invoice.id} payment failed for subscription ${subscriptionId}`);
}

function mapStripeStatus(status: string): "active" | "canceled" | "past_due" | "trialing" | "incomplete" {
  switch (status) {
    case "active":
      return "active";
    case "canceled":
      return "canceled";
    case "past_due":
      return "past_due";
    case "trialing":
      return "trialing";
    default:
      return "incomplete";
  }
}
