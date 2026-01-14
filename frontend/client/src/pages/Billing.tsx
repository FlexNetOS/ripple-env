import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Tabs, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { trpc } from "@/lib/trpc";
import { useLocation } from "wouter";
import { useEffect, useState } from "react";
import {
  Check,
  X,
  CreditCard,
  Loader2,
  Sparkles,
  Zap,
  Building2,
  ExternalLink,
  Receipt,
} from "lucide-react";
import { toast } from "sonner";

export default function Billing() {
  const [location] = useLocation();
  const [billingCycle, setBillingCycle] = useState<"monthly" | "yearly">("monthly");
  const searchParams = new URLSearchParams(location.split("?")[1] || "");

  const { data: plans, isLoading: loadingPlans } = trpc.subscription.getPlansConfig.useQuery();
  const { data: currentSubscription, isLoading: loadingSubscription } =
    trpc.subscription.getCurrentSubscription.useQuery();
  const { data: paymentHistory, isLoading: loadingHistory } =
    trpc.subscription.getPaymentHistory.useQuery();

  const createCheckout = trpc.subscription.createCheckout.useMutation({
    onSuccess: (data) => {
      if (data.url) {
        toast.info("Redirecting to checkout...");
        window.open(data.url, "_blank");
      }
    },
    onError: (error) => {
      toast.error("Failed to create checkout: " + error.message);
    },
  });

  const createPortal = trpc.subscription.createPortalSession.useMutation({
    onSuccess: (data) => {
      if (data.url) {
        window.open(data.url, "_blank");
      }
    },
    onError: (error) => {
      toast.error("Failed to open billing portal: " + error.message);
    },
  });

  // Handle success/cancel URL params
  useEffect(() => {
    if (searchParams.get("success") === "true") {
      toast.success("Payment successful! Your subscription is now active.");
    } else if (searchParams.get("canceled") === "true") {
      toast.info("Payment canceled. You can try again anytime.");
    }
  }, []);

  const handleSubscribe = (planId: string) => {
    createCheckout.mutate({ planId, billingCycle });
  };

  const getPlanIcon = (planId: string) => {
    switch (planId) {
      case "free":
        return Sparkles;
      case "pro":
        return Zap;
      case "enterprise":
        return Building2;
      default:
        return Sparkles;
    }
  };

  return (
    <div className="p-6 space-y-8">
      {/* Header */}
      <div>
        <h1 className="text-3xl font-bold tracking-tight text-white">Billing</h1>
        <p className="text-[#9BA1A6] mt-1">
          Manage your subscription and payment history
        </p>
      </div>

      {/* Current Subscription */}
      {loadingSubscription ? (
        <Card className="p-6 bg-[#1A1A1A] border-[#2A2A2A]">
          <Loader2 className="h-6 w-6 animate-spin mx-auto text-[#00D4FF]" />
        </Card>
      ) : currentSubscription ? (
        <Card className="bg-[#1A1A1A] border-[#2A2A2A]">
          <CardHeader>
            <div className="flex items-center justify-between">
              <div>
                <CardTitle className="text-white">Current Plan</CardTitle>
                <CardDescription className="text-[#6B7280]">Your active subscription</CardDescription>
              </div>
              <Badge
                variant={currentSubscription.status === "active" ? "default" : "secondary"}
                className={currentSubscription.status === "active" ? "bg-[#00E676] text-black" : ""}
              >
                {currentSubscription.status}
              </Badge>
            </div>
          </CardHeader>
          <CardContent>
            <div className="flex items-center gap-4">
              <div className="flex h-12 w-12 items-center justify-center rounded-lg bg-[#00D4FF]/10">
                <CreditCard className="h-6 w-6 text-[#00D4FF]" />
              </div>
              <div>
                <p className="font-semibold text-lg text-white">
                  {currentSubscription.plan?.name || "Unknown Plan"}
                </p>
                <p className="text-sm text-[#9BA1A6]">
                  {currentSubscription.billingCycle === "yearly" ? "Annual" : "Monthly"} billing
                  {currentSubscription.currentPeriodEnd && (
                    <> • Renews {new Date(currentSubscription.currentPeriodEnd).toLocaleDateString()}</>
                  )}
                </p>
              </div>
            </div>
          </CardContent>
          <CardFooter>
            <Button
              variant="outline"
              onClick={() => createPortal.mutate()}
              disabled={createPortal.isPending}
              className="border-[#2A2A2A] text-white hover:bg-[#252525]"
            >
              {createPortal.isPending ? (
                <Loader2 className="h-4 w-4 mr-2 animate-spin" />
              ) : (
                <ExternalLink className="h-4 w-4 mr-2" />
              )}
              Manage Subscription
            </Button>
          </CardFooter>
        </Card>
      ) : null}

      {/* Pricing Plans */}
      <div>
        <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4 mb-6">
          <h2 className="text-2xl font-semibold text-white">Subscription Plans</h2>
          <Tabs
            value={billingCycle}
            onValueChange={(v) => setBillingCycle(v as "monthly" | "yearly")}
          >
            <TabsList className="bg-[#1A1A1A]">
              <TabsTrigger value="monthly" className="data-[state=active]:bg-[#252525]">Monthly</TabsTrigger>
              <TabsTrigger value="yearly" className="data-[state=active]:bg-[#252525]">
                Yearly
                <Badge variant="secondary" className="ml-2 text-xs bg-[#00E676]/10 text-[#00E676]">
                  Save 17%
                </Badge>
              </TabsTrigger>
            </TabsList>
          </Tabs>
        </div>

        {loadingPlans ? (
          <div className="flex items-center justify-center py-12">
            <Loader2 className="h-8 w-8 animate-spin text-[#00D4FF]" />
          </div>
        ) : plans ? (
          <div className="grid gap-6 lg:grid-cols-3">
            {plans.map((plan) => {
              const PlanIcon = getPlanIcon(plan.id);
              const price = billingCycle === "monthly" ? plan.priceMonthly : plan.priceYearly;
              const priceFormatted =
                billingCycle === "monthly"
                  ? plan.priceMonthlyFormatted
                  : plan.priceYearlyFormatted;
              const isCurrentPlan = currentSubscription?.plan?.name === plan.name;

              return (
                <Card
                  key={plan.id}
                  className={`relative bg-[#1A1A1A] border-[#2A2A2A] ${
                    plan.popular ? "border-[#00D4FF] shadow-lg shadow-[#00D4FF]/10" : ""
                  }`}
                >
                  {plan.popular && (
                    <div className="absolute -top-3 left-1/2 -translate-x-1/2">
                      <Badge className="bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] text-white">Most Popular</Badge>
                    </div>
                  )}
                  <CardHeader>
                    <div className="flex items-center gap-3 mb-2">
                      <div
                        className={`flex h-10 w-10 items-center justify-center rounded-lg ${
                          plan.popular ? "bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] text-white" : "bg-[#00D4FF]/10"
                        }`}
                      >
                        <PlanIcon className={`h-5 w-5 ${plan.popular ? "" : "text-[#00D4FF]"}`} />
                      </div>
                      <div>
                        <CardTitle className="text-white">{plan.name}</CardTitle>
                      </div>
                    </div>
                    <CardDescription className="text-[#6B7280]">{plan.description}</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="mb-6">
                      <span className="text-4xl font-bold text-white">
                        {price === 0 ? "Free" : priceFormatted}
                      </span>
                      {price > 0 && (
                        <span className="text-[#6B7280]">
                          /{billingCycle === "monthly" ? "mo" : "yr"}
                        </span>
                      )}
                    </div>
                    <ul className="space-y-3">
                      {plan.features.map((feature, index) => (
                        <li key={index} className="flex items-center gap-2">
                          {feature.included ? (
                            <Check className="h-4 w-4 text-[#00E676] flex-shrink-0" />
                          ) : (
                            <X className="h-4 w-4 text-[#6B7280] flex-shrink-0" />
                          )}
                          <span
                            className={
                              feature.included ? "text-white" : "text-[#6B7280]"
                            }
                          >
                            {feature.name}
                          </span>
                        </li>
                      ))}
                    </ul>
                  </CardContent>
                  <CardFooter>
                    {isCurrentPlan ? (
                      <Button variant="outline" className="w-full border-[#2A2A2A] text-[#6B7280]" disabled>
                        Current Plan
                      </Button>
                    ) : plan.id === "free" ? (
                      <Button variant="outline" className="w-full border-[#2A2A2A] text-[#6B7280]" disabled>
                        Free Forever
                      </Button>
                    ) : (
                      <Button
                        className={`w-full ${
                          plan.popular 
                            ? "bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white" 
                            : "border-[#2A2A2A] text-white hover:bg-[#252525]"
                        }`}
                        variant={plan.popular ? "default" : "outline"}
                        onClick={() => handleSubscribe(plan.id)}
                        disabled={createCheckout.isPending}
                      >
                        {createCheckout.isPending ? (
                          <Loader2 className="h-4 w-4 mr-2 animate-spin" />
                        ) : null}
                        Subscribe
                      </Button>
                    )}
                  </CardFooter>
                </Card>
              );
            })}
          </div>
        ) : null}
      </div>

      {/* Payment History */}
      <div>
        <h2 className="text-2xl font-semibold mb-6 text-white">Payment History</h2>
        {loadingHistory ? (
          <Card className="p-6 bg-[#1A1A1A] border-[#2A2A2A]">
            <Loader2 className="h-6 w-6 animate-spin mx-auto text-[#00D4FF]" />
          </Card>
        ) : paymentHistory && paymentHistory.length > 0 ? (
          <Card className="bg-[#1A1A1A] border-[#2A2A2A]">
            <div className="divide-y divide-[#2A2A2A]">
              {paymentHistory.map((payment) => (
                <div
                  key={payment.id}
                  className="flex items-center justify-between p-4"
                >
                  <div className="flex items-center gap-4">
                    <div className="flex h-10 w-10 items-center justify-center rounded-full bg-[#252525]">
                      <Receipt className="h-5 w-5 text-[#9BA1A6]" />
                    </div>
                    <div>
                      <p className="font-medium text-white">
                        {payment.description || "Payment"}
                      </p>
                      <p className="text-sm text-[#6B7280]">
                        {new Date(payment.createdAt).toLocaleDateString()}
                      </p>
                    </div>
                  </div>
                  <div className="text-right">
                    <p className="font-semibold text-white">
                      ${(payment.amount / 100).toFixed(2)} {payment.currency.toUpperCase()}
                    </p>
                    <Badge
                      variant={payment.status === "succeeded" ? "default" : "secondary"}
                      className={payment.status === "succeeded" ? "bg-[#00E676] text-black" : ""}
                    >
                      {payment.status}
                    </Badge>
                  </div>
                </div>
              ))}
            </div>
          </Card>
        ) : (
          <Card className="py-12 bg-[#1A1A1A] border-[#2A2A2A]">
            <div className="text-center">
              <Receipt className="h-16 w-16 mx-auto mb-4 text-[#9B7BFF] opacity-50" />
              <h3 className="text-lg font-medium mb-2 text-white">No payment history</h3>
              <p className="text-[#6B7280]">
                Your payment history will appear here
              </p>
            </div>
          </Card>
        )}
      </div>

      {/* Test Card Info */}
      <Card className="bg-[#1A1A1A]/50 border-[#2A2A2A]">
        <CardContent className="flex items-center gap-4 p-4">
          <CreditCard className="h-5 w-5 text-[#9BA1A6]" />
          <p className="text-sm text-[#9BA1A6]">
            <strong className="text-white">Test Mode:</strong> Use card number{" "}
            <code className="bg-[#252525] px-1 py-0.5 rounded text-[#00D4FF]">4242 4242 4242 4242</code>{" "}
            with any future expiry date and CVC to test payments.
          </p>
        </CardContent>
      </Card>
    </div>
  );
}
