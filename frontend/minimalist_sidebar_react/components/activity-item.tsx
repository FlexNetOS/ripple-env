import { View, Text } from "react-native";
import { IconSymbol } from "@/components/ui/icon-symbol";
import { useColors } from "@/hooks/use-colors";

type ActivityType = "success" | "warning" | "error" | "info";

interface ActivityItemProps {
  type: ActivityType;
  message: string;
  time: string;
  isLast?: boolean;
}

export function ActivityItem({ type, message, time, isLast = false }: ActivityItemProps) {
  const colors = useColors();
  
  const typeConfig: Record<ActivityType, { color: string; icon: "checkmark.circle.fill" | "exclamationmark.triangle.fill" | "xmark.circle.fill" | "info.circle.fill" }> = {
    success: { color: colors.success, icon: "checkmark.circle.fill" },
    warning: { color: colors.warning, icon: "exclamationmark.triangle.fill" },
    error: { color: colors.error, icon: "xmark.circle.fill" },
    info: { color: colors.primary, icon: "info.circle.fill" },
  };

  const config = typeConfig[type];

  return (
    <View 
      className="flex-row items-center p-4 gap-3"
      style={{ 
        borderBottomWidth: isLast ? 0 : 1,
        borderBottomColor: colors.border,
      }}
    >
      <IconSymbol name={config.icon} size={20} color={config.color} />
      <View className="flex-1">
        <Text className="text-sm text-foreground">{message}</Text>
      </View>
      <Text className="text-xs text-muted">{time}</Text>
    </View>
  );
}
