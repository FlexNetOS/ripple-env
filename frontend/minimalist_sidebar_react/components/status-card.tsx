import { View, Text } from "react-native";
import { IconSymbol } from "@/components/ui/icon-symbol";
import { useColors } from "@/hooks/use-colors";

type StatusType = "success" | "warning" | "error" | "info";

interface StatusCardProps {
  title: string;
  value: string;
  subtitle: string;
  status: StatusType;
  icon: "person.2.fill" | "bolt.fill" | "hammer.fill" | "server.rack" | "checkmark.circle.fill";
}

export function StatusCard({ title, value, subtitle, status, icon }: StatusCardProps) {
  const colors = useColors();
  
  const statusColors: Record<StatusType, string> = {
    success: colors.success,
    warning: colors.warning,
    error: colors.error,
    info: colors.primary,
  };

  const statusColor = statusColors[status];

  return (
    <View 
      className="flex-1 min-w-[140px] p-4 rounded-xl"
      style={{ backgroundColor: colors.surface }}
    >
      <View className="flex-row items-center justify-between mb-2">
        <IconSymbol name={icon} size={20} color={statusColor} />
        <View 
          className="w-2 h-2 rounded-full"
          style={{ backgroundColor: statusColor }}
        />
      </View>
      <Text className="text-2xl font-bold text-foreground">{value}</Text>
      <Text className="text-xs text-muted mt-1">{title}</Text>
      <Text className="text-xs mt-0.5" style={{ color: statusColor }}>{subtitle}</Text>
    </View>
  );
}
