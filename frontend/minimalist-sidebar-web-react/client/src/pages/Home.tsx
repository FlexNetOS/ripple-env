import { useAuth } from "@/_core/hooks/useAuth";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { getLoginUrl } from "@/const";
import { Link } from "wouter";
import {
  MessageSquare,
  FileUp,
  Bell,
  CreditCard,
  Shield,
  Sparkles,
  ArrowRight,
  CheckCircle2,
  Zap,
  Users,
  Lock,
} from "lucide-react";

export default function Home() {
  const { user, loading, isAuthenticated } = useAuth();

  const features = [
    {
      icon: MessageSquare,
      title: "AI Chat Assistant",
      description: "Intelligent conversations powered by advanced language models for natural interactions.",
      color: "from-[#00D4FF] to-[#00D4FF]/60",
    },
    {
      icon: FileUp,
      title: "Secure File Storage",
      description: "Upload and manage your files with enterprise-grade S3 storage integration.",
      color: "from-[#9B7BFF] to-[#9B7BFF]/60",
    },
    {
      icon: Bell,
      title: "Smart Notifications",
      description: "Stay informed with real-time alerts and customizable notification preferences.",
      color: "from-[#00E676] to-[#00E676]/60",
    },
    {
      icon: CreditCard,
      title: "Flexible Billing",
      description: "Secure payment processing with Stripe for seamless subscription management.",
      color: "from-[#00D4FF] to-[#9B7BFF]",
    },
    {
      icon: Shield,
      title: "Role-Based Access",
      description: "Fine-grained permissions with admin and user roles for secure collaboration.",
      color: "from-[#9B7BFF] to-[#00E676]",
    },
    {
      icon: Sparkles,
      title: "Modern Interface",
      description: "Beautiful, responsive design with elegant animations and intuitive navigation.",
      color: "from-[#00E676] to-[#00D4FF]",
    },
  ];

  const benefits = [
    "Type-safe API with tRPC",
    "Real-time data synchronization",
    "Secure authentication with OAuth",
    "Production-ready infrastructure",
  ];

  return (
    <div className="min-h-screen bg-[#0D0D0D]">
      {/* Navigation */}
      <nav className="fixed top-0 left-0 right-0 z-50 border-b border-[#1A1A1A] bg-[#0D0D0D]/80 backdrop-blur-lg">
        <div className="container flex h-16 items-center justify-between">
          <div className="flex items-center gap-3">
            <img 
              src="/images/icon.png" 
              alt="Ripple" 
              className="h-10 w-10 object-contain"
            />
            <span className="text-xl font-semibold tracking-tight bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] bg-clip-text text-transparent">
              Ripple
            </span>
          </div>
          <div className="flex items-center gap-4">
            {loading ? (
              <div className="h-9 w-24 animate-pulse rounded-md bg-[#1A1A1A]" />
            ) : isAuthenticated ? (
              <Link href="/dashboard">
                <Button className="bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white">
                  Dashboard
                  <ArrowRight className="ml-2 h-4 w-4" />
                </Button>
              </Link>
            ) : (
              <>
                <a href={getLoginUrl()}>
                  <Button variant="ghost" className="text-[#9BA1A6] hover:text-white hover:bg-[#1A1A1A]">
                    Sign In
                  </Button>
                </a>
                <a href={getLoginUrl()}>
                  <Button className="bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white">
                    Get Started
                    <ArrowRight className="ml-2 h-4 w-4" />
                  </Button>
                </a>
              </>
            )}
          </div>
        </div>
      </nav>

      {/* Hero Section */}
      <section className="relative overflow-hidden pt-32 pb-20">
        <div className="absolute inset-0 -z-10">
          <div className="absolute top-0 left-1/4 h-[500px] w-[500px] rounded-full bg-[#00D4FF]/10 blur-[100px]" />
          <div className="absolute bottom-0 right-1/4 h-[400px] w-[400px] rounded-full bg-[#9B7BFF]/10 blur-[80px]" />
          <div className="absolute top-1/2 right-1/3 h-[300px] w-[300px] rounded-full bg-[#00E676]/5 blur-[60px]" />
        </div>

        <div className="container">
          <div className="mx-auto max-w-3xl text-center">
            <div className="mb-6 inline-flex items-center gap-2 rounded-full border border-[#2A2A2A] bg-[#1A1A1A] px-4 py-1.5 text-sm">
              <Zap className="h-4 w-4 text-[#00D4FF]" />
              <span className="text-[#9BA1A6]">Production-ready full-stack platform</span>
            </div>
            <h1 className="mb-6 text-5xl font-bold tracking-tight sm:text-6xl lg:text-7xl text-white">
              Build Faster with{" "}
              <span className="bg-gradient-to-r from-[#00D4FF] via-[#9B7BFF] to-[#00E676] bg-clip-text text-transparent">
                Intelligent Tools
              </span>
            </h1>
            <p className="mb-8 text-lg text-[#9BA1A6] sm:text-xl">
              A comprehensive platform combining AI-powered features, secure file management,
              and seamless payment processing. Everything you need to build modern applications.
            </p>
            <div className="flex flex-col items-center justify-center gap-4 sm:flex-row">
              <a href={getLoginUrl()}>
                <Button size="lg" className="h-12 px-8 text-base bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white">
                  Start Building
                  <ArrowRight className="ml-2 h-5 w-5" />
                </Button>
              </a>
              <Link href="/pricing">
                <Button variant="outline" size="lg" className="h-12 px-8 text-base border-[#2A2A2A] text-white hover:bg-[#1A1A1A]">
                  View Pricing
                </Button>
              </Link>
            </div>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className="py-20">
        <div className="container">
          <div className="mx-auto mb-16 max-w-2xl text-center">
            <h2 className="mb-4 text-3xl font-bold tracking-tight sm:text-4xl text-white">
              Everything You Need
            </h2>
            <p className="text-lg text-[#9BA1A6]">
              Powerful features designed to accelerate your development workflow
              and deliver exceptional user experiences.
            </p>
          </div>
          <div className="grid gap-6 sm:grid-cols-2 lg:grid-cols-3">
            {features.map((feature, index) => (
              <Card
                key={index}
                className="group relative overflow-hidden border-[#2A2A2A] bg-[#1A1A1A]/50 transition-all duration-300 hover:border-[#00D4FF]/30 hover:shadow-lg hover:shadow-[#00D4FF]/5"
              >
                <CardHeader>
                  <div className={`mb-4 flex h-12 w-12 items-center justify-center rounded-lg bg-gradient-to-r ${feature.color} text-white transition-transform group-hover:scale-110`}>
                    <feature.icon className="h-6 w-6" />
                  </div>
                  <CardTitle className="text-xl text-white">{feature.title}</CardTitle>
                </CardHeader>
                <CardContent>
                  <CardDescription className="text-base text-[#6B7280]">
                    {feature.description}
                  </CardDescription>
                </CardContent>
              </Card>
            ))}
          </div>
        </div>
      </section>

      {/* Benefits Section */}
      <section className="border-y border-[#1A1A1A] bg-[#0A0A0A] py-20">
        <div className="container">
          <div className="grid items-center gap-12 lg:grid-cols-2">
            <div>
              <h2 className="mb-4 text-3xl font-bold tracking-tight sm:text-4xl text-white">
                Built for Scale
              </h2>
              <p className="mb-8 text-lg text-[#9BA1A6]">
                Enterprise-grade architecture with modern technologies that grow with your needs.
                From startup to scale-up, we've got you covered.
              </p>
              <ul className="space-y-4">
                {benefits.map((benefit, index) => (
                  <li key={index} className="flex items-center gap-3">
                    <CheckCircle2 className="h-5 w-5 flex-shrink-0 text-[#00E676]" />
                    <span className="text-white">{benefit}</span>
                  </li>
                ))}
              </ul>
            </div>
            <div className="relative">
              <div className="aspect-square rounded-2xl border border-[#2A2A2A] bg-gradient-to-br from-[#00D4FF]/5 via-[#9B7BFF]/5 to-[#00E676]/5 p-8">
                <div className="flex h-full flex-col items-center justify-center gap-6">
                  <div className="flex items-center gap-4">
                    <div className="flex h-16 w-16 items-center justify-center rounded-xl bg-[#00D4FF]/20">
                      <Users className="h-8 w-8 text-[#00D4FF]" />
                    </div>
                    <div className="flex h-16 w-16 items-center justify-center rounded-xl bg-[#9B7BFF]/20">
                      <Lock className="h-8 w-8 text-[#9B7BFF]" />
                    </div>
                    <div className="flex h-16 w-16 items-center justify-center rounded-xl bg-[#00E676]/20">
                      <Zap className="h-8 w-8 text-[#00E676]" />
                    </div>
                  </div>
                  <div className="text-center">
                    <p className="text-4xl font-bold bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] bg-clip-text text-transparent">99.9%</p>
                    <p className="text-sm text-[#6B7280]">Uptime Guarantee</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="py-20">
        <div className="container">
          <div className="relative overflow-hidden rounded-3xl bg-gradient-to-r from-[#00D4FF] via-[#9B7BFF] to-[#00E676] px-8 py-16 text-center sm:px-16">
            <div className="absolute inset-0 bg-[url('data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iNjAiIGhlaWdodD0iNjAiIHZpZXdCb3g9IjAgMCA2MCA2MCIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj48ZyBmaWxsPSJub25lIiBmaWxsLXJ1bGU9ImV2ZW5vZGQiPjxnIGZpbGw9IiNmZmYiIGZpbGwtb3BhY2l0eT0iMC4xIj48cGF0aCBkPSJNMzYgMzRoLTJ2LTRoMnY0em0wLThoLTJ2LTRoMnY0em0tOCA4aC0ydi00aDJ2NHptMC04aC0ydi00aDJ2NHoiLz48L2c+PC9nPjwvc3ZnPg==')] opacity-30" />
            <div className="relative">
              <h2 className="mb-4 text-3xl font-bold tracking-tight text-white sm:text-4xl">
                Ready to Get Started?
              </h2>
              <p className="mx-auto mb-8 max-w-2xl text-lg text-white/80">
                Join thousands of developers building amazing applications with our platform.
                Start your free trial today.
              </p>
              <a href={getLoginUrl()}>
                <Button size="lg" variant="secondary" className="h-12 px-8 text-base bg-white text-[#0D0D0D] hover:bg-white/90">
                  Create Free Account
                  <ArrowRight className="ml-2 h-5 w-5" />
                </Button>
              </a>
            </div>
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="border-t border-[#1A1A1A] py-12">
        <div className="container">
          <div className="flex flex-col items-center justify-between gap-4 sm:flex-row">
            <div className="flex items-center gap-3">
              <img 
                src="/images/icon.png" 
                alt="Ripple" 
                className="h-8 w-8 object-contain"
              />
              <span className="font-semibold bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] bg-clip-text text-transparent">
                Ripple
              </span>
            </div>
            <p className="text-sm text-[#6B7280]">
              © {new Date().getFullYear()} Ripple. All rights reserved.
            </p>
          </div>
        </div>
      </footer>
    </div>
  );
}
