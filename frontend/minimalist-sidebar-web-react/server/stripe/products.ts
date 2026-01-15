/**
 * Stripe Products and Pricing Configuration
 * 
 * Products and prices are defined here for centralized access.
 * In production, these should be created in Stripe Dashboard and IDs stored here.
 */

export interface PlanFeature {
  name: string;
  included: boolean;
}

export interface SubscriptionPlan {
  id: string;
  name: string;
  description: string;
  priceMonthly: number; // in cents
  priceYearly: number; // in cents (typically with discount)
  features: PlanFeature[];
  popular?: boolean;
  stripePriceIdMonthly?: string;
  stripePriceIdYearly?: string;
}

export const SUBSCRIPTION_PLANS: SubscriptionPlan[] = [
  {
    id: "free",
    name: "Free",
    description: "Perfect for getting started",
    priceMonthly: 0,
    priceYearly: 0,
    features: [
      { name: "5 AI conversations per day", included: true },
      { name: "100MB file storage", included: true },
      { name: "Basic notifications", included: true },
      { name: "Email support", included: true },
      { name: "Priority support", included: false },
      { name: "Advanced analytics", included: false },
      { name: "Custom integrations", included: false },
    ],
  },
  {
    id: "pro",
    name: "Pro",
    description: "Best for professionals",
    priceMonthly: 1999, // $19.99
    priceYearly: 19990, // $199.90 (save ~17%)
    popular: true,
    features: [
      { name: "Unlimited AI conversations", included: true },
      { name: "10GB file storage", included: true },
      { name: "Advanced notifications", included: true },
      { name: "Priority email support", included: true },
      { name: "Priority support", included: true },
      { name: "Advanced analytics", included: true },
      { name: "Custom integrations", included: false },
    ],
  },
  {
    id: "enterprise",
    name: "Enterprise",
    description: "For teams and organizations",
    priceMonthly: 4999, // $49.99
    priceYearly: 49990, // $499.90 (save ~17%)
    features: [
      { name: "Unlimited AI conversations", included: true },
      { name: "Unlimited file storage", included: true },
      { name: "Advanced notifications", included: true },
      { name: "24/7 priority support", included: true },
      { name: "Priority support", included: true },
      { name: "Advanced analytics", included: true },
      { name: "Custom integrations", included: true },
    ],
  },
];

export function getPlanById(planId: string): SubscriptionPlan | undefined {
  return SUBSCRIPTION_PLANS.find((plan) => plan.id === planId);
}

export function formatPrice(cents: number): string {
  return new Intl.NumberFormat("en-US", {
    style: "currency",
    currency: "USD",
  }).format(cents / 100);
}
