import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { trpc } from "@/lib/trpc";
import { Link } from "wouter";
import {
  MessageSquare,
  FileUp,
  Bell,
  CreditCard,
  ArrowRight,
  Clock,
  Sparkles,
} from "lucide-react";

export default function Dashboard() {
  const { data: conversations } = trpc.chat.listConversations.useQuery();
  const { data: files } = trpc.files.list.useQuery();
  const { data: notifications } = trpc.notifications.list.useQuery({});
  const { data: unreadCount } = trpc.notifications.unreadCount.useQuery();

  const stats = [
    {
      title: "Conversations",
      value: conversations?.length ?? 0,
      description: "AI chat sessions",
      icon: MessageSquare,
      href: "/chat",
      color: "text-[#00D4FF]",
      bgColor: "bg-[#00D4FF]/10",
    },
    {
      title: "Files",
      value: files?.length ?? 0,
      description: "Uploaded files",
      icon: FileUp,
      href: "/files",
      color: "text-[#00E676]",
      bgColor: "bg-[#00E676]/10",
    },
    {
      title: "Notifications",
      value: unreadCount ?? 0,
      description: "Unread alerts",
      icon: Bell,
      href: "/notifications",
      color: "text-[#9B7BFF]",
      bgColor: "bg-[#9B7BFF]/10",
    },
  ];

  const quickActions = [
    {
      title: "New Chat",
      description: "Start a conversation with AI",
      icon: MessageSquare,
      href: "/chat",
    },
    {
      title: "Upload File",
      description: "Add files to your storage",
      icon: FileUp,
      href: "/files",
    },
    {
      title: "View Billing",
      description: "Manage your subscription",
      icon: CreditCard,
      href: "/billing",
    },
  ];

  return (
    <div className="p-6 space-y-8">
      {/* Header */}
      <div>
        <h1 className="text-3xl font-bold tracking-tight text-white">Dashboard</h1>
        <p className="text-[#9BA1A6] mt-1">
          Welcome back! Here's an overview of your activity.
        </p>
      </div>

      {/* Stats Grid */}
      <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
        {stats.map((stat) => (
          <Link key={stat.title} href={stat.href}>
            <Card className="group cursor-pointer transition-all hover:shadow-md hover:border-[#00D4FF]/30 bg-[#1A1A1A] border-[#2A2A2A]">
              <CardHeader className="flex flex-row items-center justify-between pb-2">
                <CardTitle className="text-sm font-medium text-[#9BA1A6]">
                  {stat.title}
                </CardTitle>
                <div className={`rounded-lg p-2 ${stat.bgColor}`}>
                  <stat.icon className={`h-4 w-4 ${stat.color}`} />
                </div>
              </CardHeader>
              <CardContent>
                <div className="text-3xl font-bold text-white">{stat.value}</div>
                <p className="text-xs text-[#6B7280] mt-1">
                  {stat.description}
                </p>
              </CardContent>
            </Card>
          </Link>
        ))}
      </div>

      {/* Quick Actions */}
      <div>
        <h2 className="text-xl font-semibold mb-4 text-white">Quick Actions</h2>
        <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {quickActions.map((action) => (
            <Link key={action.title} href={action.href}>
              <Card className="group cursor-pointer transition-all hover:shadow-md hover:border-[#00D4FF]/30 bg-[#1A1A1A] border-[#2A2A2A]">
                <CardContent className="flex items-center gap-4 p-6">
                  <div className="flex h-12 w-12 items-center justify-center rounded-lg bg-[#00D4FF]/10 text-[#00D4FF] group-hover:bg-[#00D4FF] group-hover:text-black transition-colors">
                    <action.icon className="h-6 w-6" />
                  </div>
                  <div className="flex-1">
                    <h3 className="font-semibold text-white">{action.title}</h3>
                    <p className="text-sm text-[#9BA1A6]">
                      {action.description}
                    </p>
                  </div>
                  <ArrowRight className="h-5 w-5 text-[#6B7280] group-hover:text-[#00D4FF] transition-colors" />
                </CardContent>
              </Card>
            </Link>
          ))}
        </div>
      </div>

      {/* Recent Activity */}
      <div className="grid gap-6 lg:grid-cols-2">
        {/* Recent Conversations */}
        <Card className="bg-[#1A1A1A] border-[#2A2A2A]">
          <CardHeader>
            <CardTitle className="flex items-center gap-2 text-white">
              <MessageSquare className="h-5 w-5 text-[#00D4FF]" />
              Recent Conversations
            </CardTitle>
            <CardDescription className="text-[#6B7280]">Your latest AI chat sessions</CardDescription>
          </CardHeader>
          <CardContent>
            {conversations && conversations.length > 0 ? (
              <div className="space-y-3">
                {conversations.slice(0, 5).map((conv) => (
                  <Link key={conv.id} href={`/chat?id=${conv.id}`}>
                    <div className="flex items-center gap-3 rounded-lg p-3 hover:bg-[#252525] transition-colors cursor-pointer">
                      <div className="flex h-10 w-10 items-center justify-center rounded-full bg-[#00D4FF]/10">
                        <Sparkles className="h-5 w-5 text-[#00D4FF]" />
                      </div>
                      <div className="flex-1 min-w-0">
                        <p className="font-medium truncate text-white">{conv.title}</p>
                        <p className="text-xs text-[#6B7280] flex items-center gap-1">
                          <Clock className="h-3 w-3" />
                          {new Date(conv.updatedAt).toLocaleDateString()}
                        </p>
                      </div>
                    </div>
                  </Link>
                ))}
              </div>
            ) : (
              <div className="text-center py-8 text-[#6B7280]">
                <MessageSquare className="h-12 w-12 mx-auto mb-3 opacity-50" />
                <p>No conversations yet</p>
                <Link href="/chat">
                  <Button variant="link" className="mt-2 text-[#00D4FF]">
                    Start your first chat
                  </Button>
                </Link>
              </div>
            )}
          </CardContent>
        </Card>

        {/* Recent Notifications */}
        <Card className="bg-[#1A1A1A] border-[#2A2A2A]">
          <CardHeader>
            <CardTitle className="flex items-center gap-2 text-white">
              <Bell className="h-5 w-5 text-[#9B7BFF]" />
              Recent Notifications
            </CardTitle>
            <CardDescription className="text-[#6B7280]">Your latest alerts and updates</CardDescription>
          </CardHeader>
          <CardContent>
            {notifications && notifications.length > 0 ? (
              <div className="space-y-3">
                {notifications.slice(0, 5).map((notif) => (
                  <div
                    key={notif.id}
                    className={`flex items-start gap-3 rounded-lg p-3 ${
                      notif.isRead ? "opacity-60" : "bg-[#252525]"
                    }`}
                  >
                    <div
                      className={`flex h-8 w-8 items-center justify-center rounded-full ${
                        notif.type === "success"
                          ? "bg-[#00E676]/10 text-[#00E676]"
                          : notif.type === "warning"
                          ? "bg-yellow-500/10 text-yellow-500"
                          : notif.type === "error"
                          ? "bg-red-500/10 text-red-500"
                          : "bg-[#00D4FF]/10 text-[#00D4FF]"
                      }`}
                    >
                      <Bell className="h-4 w-4" />
                    </div>
                    <div className="flex-1 min-w-0">
                      <p className="font-medium text-sm text-white">{notif.title}</p>
                      <p className="text-xs text-[#6B7280] truncate">
                        {notif.content}
                      </p>
                    </div>
                  </div>
                ))}
              </div>
            ) : (
              <div className="text-center py-8 text-[#6B7280]">
                <Bell className="h-12 w-12 mx-auto mb-3 opacity-50" />
                <p>No notifications yet</p>
              </div>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  );
}
