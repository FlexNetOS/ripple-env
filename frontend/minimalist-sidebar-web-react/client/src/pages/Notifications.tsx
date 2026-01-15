import { Button } from "@/components/ui/button";
import { Card, CardContent } from "@/components/ui/card";
import { trpc } from "@/lib/trpc";
import { Link } from "wouter";
import {
  Bell,
  BellOff,
  Check,
  CheckCheck,
  Info,
  AlertTriangle,
  AlertCircle,
  CheckCircle,
  Loader2,
  ExternalLink,
} from "lucide-react";
import { toast } from "sonner";

function getNotificationIcon(type: string) {
  switch (type) {
    case "success":
      return { icon: CheckCircle, color: "text-[#00E676]", bg: "bg-[#00E676]/10" };
    case "warning":
      return { icon: AlertTriangle, color: "text-yellow-500", bg: "bg-yellow-500/10" };
    case "error":
      return { icon: AlertCircle, color: "text-red-500", bg: "bg-red-500/10" };
    default:
      return { icon: Info, color: "text-[#00D4FF]", bg: "bg-[#00D4FF]/10" };
  }
}

export default function Notifications() {
  const utils = trpc.useUtils();

  const { data: notifications, isLoading } = trpc.notifications.list.useQuery({});
  const { data: unreadCount } = trpc.notifications.unreadCount.useQuery();

  const markAsRead = trpc.notifications.markAsRead.useMutation({
    onSuccess: () => {
      utils.notifications.list.invalidate();
      utils.notifications.unreadCount.invalidate();
    },
  });

  const markAllAsRead = trpc.notifications.markAllAsRead.useMutation({
    onSuccess: () => {
      utils.notifications.list.invalidate();
      utils.notifications.unreadCount.invalidate();
      toast.success("All notifications marked as read");
    },
  });

  const handleMarkAsRead = (id: number) => {
    markAsRead.mutate({ id });
  };

  return (
    <div className="p-6 space-y-6">
      {/* Header */}
      <div className="flex flex-col sm:flex-row sm:items-center sm:justify-between gap-4">
        <div>
          <h1 className="text-3xl font-bold tracking-tight text-white">Notifications</h1>
          <p className="text-[#9BA1A6] mt-1">
            {unreadCount ? `${unreadCount} unread` : "All caught up!"}
          </p>
        </div>
        {notifications && notifications.length > 0 && unreadCount && unreadCount > 0 && (
          <Button
            variant="outline"
            onClick={() => markAllAsRead.mutate()}
            disabled={markAllAsRead.isPending}
            className="border-[#2A2A2A] text-white hover:bg-[#252525]"
          >
            {markAllAsRead.isPending ? (
              <Loader2 className="h-4 w-4 mr-2 animate-spin" />
            ) : (
              <CheckCheck className="h-4 w-4 mr-2" />
            )}
            Mark All as Read
          </Button>
        )}
      </div>

      {/* Notifications List */}
      {isLoading ? (
        <div className="flex items-center justify-center py-12">
          <Loader2 className="h-8 w-8 animate-spin text-[#00D4FF]" />
        </div>
      ) : notifications && notifications.length > 0 ? (
        <div className="space-y-3">
          {notifications.map((notif) => {
            const { icon: Icon, color, bg } = getNotificationIcon(notif.type);
            return (
              <Card
                key={notif.id}
                className={`transition-all bg-[#1A1A1A] border-[#2A2A2A] ${
                  notif.isRead ? "opacity-60" : "border-[#00D4FF]/30 shadow-sm"
                }`}
              >
                <CardContent className="flex items-start gap-4 p-4">
                  <div className={`flex h-10 w-10 items-center justify-center rounded-full ${bg} flex-shrink-0`}>
                    <Icon className={`h-5 w-5 ${color}`} />
                  </div>
                  <div className="flex-1 min-w-0">
                    <div className="flex items-start justify-between gap-2">
                      <div>
                        <h3 className="font-medium text-white">{notif.title}</h3>
                        <p className="text-sm text-[#9BA1A6] mt-1">
                          {notif.content}
                        </p>
                        <p className="text-xs text-[#6B7280] mt-2">
                          {new Date(notif.createdAt).toLocaleString()}
                        </p>
                      </div>
                      <div className="flex items-center gap-2 flex-shrink-0">
                        {notif.link && (
                          <Link href={notif.link}>
                            <Button variant="ghost" size="icon" className="h-8 w-8 text-[#9BA1A6] hover:text-white">
                              <ExternalLink className="h-4 w-4" />
                            </Button>
                          </Link>
                        )}
                        {!notif.isRead && (
                          <Button
                            variant="ghost"
                            size="icon"
                            className="h-8 w-8 text-[#00D4FF] hover:text-[#00D4FF] hover:bg-[#00D4FF]/10"
                            onClick={() => handleMarkAsRead(notif.id)}
                            disabled={markAsRead.isPending}
                          >
                            <Check className="h-4 w-4" />
                          </Button>
                        )}
                      </div>
                    </div>
                  </div>
                </CardContent>
              </Card>
            );
          })}
        </div>
      ) : (
        <Card className="py-12 bg-[#1A1A1A] border-[#2A2A2A]">
          <div className="text-center">
            <BellOff className="h-16 w-16 mx-auto mb-4 text-[#9B7BFF] opacity-50" />
            <h3 className="text-lg font-medium mb-2 text-white">No notifications</h3>
            <p className="text-[#6B7280]">
              You're all caught up! New notifications will appear here.
            </p>
          </div>
        </Card>
      )}
    </div>
  );
}
