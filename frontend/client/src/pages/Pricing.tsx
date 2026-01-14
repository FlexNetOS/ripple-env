import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Badge } from "@/components/ui/badge";
import { Tabs, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { trpc } from "@/lib/trpc";
import { Link } from "wouter";
import { useState } from "react";
import { getLoginUrl } from "@/const";
import { useAuth } from "@/_core/hooks/useAuth";
import {
  Check,
  X,
  Sparkles,
  Zap,
  Building2,
  ArrowRight,
  ArrowLeft,
  Loader2,
} from "lucide-react";

export default function Pricing() {
  const [billingCycle, setBillingCycle] = useState<"monthly" | "yearly">("monthly");
  const { isAuthenticated } = useAuth();

  const { data: plans, isLoading } = trpc.subscription.getPlansConfig.useQuery();

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
    <div className="min-h-screen bg-background">
      {/* Navigation */}
      <nav className="border-b border-border/50 bg-background/80 backdrop-blur-lg sticky top-0 z-50">
        <div className="container flex h-16 items-center justify-between">
          <Link href="/">
            <Button variant="ghost" className="gap-2">
              <ArrowLeft className="h-4 w-4" />
              Back to Home
            </Button>
          </Link>
          {isAuthenticated ? (
            <Link href="/dashboard">
              <Button>
                Dashboard
                <ArrowRight className="ml-2 h-4 w-4" />
              </Button>
            </Link>
          ) : (
            <a href={getLoginUrl()}>
              <Button>
                Get Started
                <ArrowRight className="ml-2 h-4 w-4" />
              </Button>
            </a>
          )}
        </div>
      </nav>

      {/* Header */}
      <section className="py-16 text-center">
        <div className="container">
          <h1 className="text-4xl font-bold tracking-tight sm:text-5xl mb-4">
            Simple, Transparent Pricing
          </h1>
          <p className="text-lg text-muted-foreground max-w-2xl mx-auto mb-8">
            Choose the plan that fits your needs. All plans include core features with
            no hidden fees.
          </p>
          <Tabs
            value={billingCycle}
            onValueChange={(v) => setBillingCycle(v as "monthly" | "yearly")}
            className="inline-flex"
          >
            <TabsList>
              <TabsTrigger value="monthly">Monthly</TabsTrigger>
              <TabsTrigger value="yearly" className="gap-2">
                Yearly
                <Badge variant="secondary" className="text-xs">
                  Save 17%
                </Badge>
              </TabsTrigger>
            </TabsList>
          </Tabs>
        </div>
      </section>

      {/* Pricing Cards */}
      <section className="pb-20">
        <div className="container">
          {isLoading ? (
            <div className="flex items-center justify-center py-12">
              <Loader2 className="h-8 w-8 animate-spin text-muted-foreground" />
            </div>
          ) : plans ? (
            <div className="grid gap-8 lg:grid-cols-3 max-w-5xl mx-auto">
              {plans.map((plan) => {
                const PlanIcon = getPlanIcon(plan.id);
                const price = billingCycle === "monthly" ? plan.priceMonthly : plan.priceYearly;
                const priceFormatted =
                  billingCycle === "monthly"
                    ? plan.priceMonthlyFormatted
                    : plan.priceYearlyFormatted;

                return (
                  <Card
                    key={plan.id}
                    className={`relative flex flex-col ${
                      plan.popular ? "border-primary shadow-xl scale-105" : ""
                    }`}
                  >
                    {plan.popular && (
                      <div className="absolute -top-4 left-1/2 -translate-x-1/2">
                        <Badge className="bg-primary px-4 py-1">Most Popular</Badge>
                      </div>
                    )}
                    <CardHeader className="text-center pb-2">
                      <div
                        className={`mx-auto mb-4 flex h-14 w-14 items-center justify-center rounded-xl ${
                          plan.popular ? "bg-primary text-primary-foreground" : "bg-primary/10"
                        }`}
                      >
                        <PlanIcon className={`h-7 w-7 ${plan.popular ? "" : "text-primary"}`} />
                      </div>
                      <CardTitle className="text-2xl">{plan.name}</CardTitle>
                      <CardDescription className="min-h-[40px]">
                        {plan.description}
                      </CardDescription>
                    </CardHeader>
                    <CardContent className="flex-1">
                      <div className="text-center mb-8">
                        <span className="text-5xl font-bold">
                          {price === 0 ? "Free" : priceFormatted}
                        </span>
                        {price > 0 && (
                          <span className="text-muted-foreground">
                            /{billingCycle === "monthly" ? "month" : "year"}
                          </span>
                        )}
                      </div>
                      <ul className="space-y-4">
                        {plan.features.map((feature, index) => (
                          <li key={index} className="flex items-start gap-3">
                            {feature.included ? (
                              <Check className="h-5 w-5 text-primary flex-shrink-0 mt-0.5" />
                            ) : (
                              <X className="h-5 w-5 text-muted-foreground/50 flex-shrink-0 mt-0.5" />
                            )}
                            <span
                              className={
                                feature.included ? "" : "text-muted-foreground/50"
                              }
                            >
                              {feature.name}
                            </span>
                          </li>
                        ))}
                      </ul>
                    </CardContent>
                    <CardFooter>
                      {isAuthenticated ? (
                        <Link href="/billing" className="w-full">
                          <Button
                            className="w-full"
                            variant={plan.popular ? "default" : "outline"}
                            size="lg"
                          >
                            {plan.id === "free" ? "Get Started" : "Subscribe"}
                          </Button>
                        </Link>
                      ) : (
                        <a href={getLoginUrl()} className="w-full">
                          <Button
                            className="w-full"
                            variant={plan.popular ? "default" : "outline"}
                            size="lg"
                          >
                            Get Started
                            <ArrowRight className="ml-2 h-4 w-4" />
                          </Button>
                        </a>
                      )}
                    </CardFooter>
                  </Card>
                );
              })}
            </div>
          ) : null}
        </div>
      </section>

      {/* FAQ Section */}
      <section className="border-t border-border/50 py-20 bg-muted/30">
        <div className="container max-w-3xl">
          <h2 className="text-3xl font-bold text-center mb-12">
            Frequently Asked Questions
          </h2>
          <div className="space-y-6">
            {[
              {
                q: "Can I change my plan later?",
                a: "Yes, you can upgrade or downgrade your plan at any time. Changes take effect immediately, and we'll prorate any differences.",
              },
              {
                q: "What payment methods do you accept?",
                a: "We accept all major credit cards (Visa, MasterCard, American Express) through our secure Stripe integration.",
              },
              {
                q: "Is there a free trial?",
                a: "Yes! Our Free plan gives you access to core features forever. You can upgrade to a paid plan whenever you're ready.",
              },
              {
                q: "Can I cancel my subscription?",
                a: "Absolutely. You can cancel your subscription at any time from your billing settings. You'll continue to have access until the end of your billing period.",
              },
            ].map((faq, index) => (
              <Card key={index}>
                <CardHeader>
                  <CardTitle className="text-lg">{faq.q}</CardTitle>
                </CardHeader>
                <CardContent>
                  <p className="text-muted-foreground">{faq.a}</p>
                </CardContent>
              </Card>
            ))}
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="border-t border-border/50 py-8">
        <div className="container text-center">
          <p className="text-sm text-muted-foreground">
            © {new Date().getFullYear()} Nexus. All rights reserved.
          </p>
        </div>
      </footer>
    </div>
  );
}
