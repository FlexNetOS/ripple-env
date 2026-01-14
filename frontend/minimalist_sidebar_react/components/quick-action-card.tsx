import { View, Text, Pressable } from "react-native";
import { IconSymbol } from "@/components/ui/icon-symbol";
import { useColors } from "@/hooks/use-colors";

interface QuickActionCardProps {
  title: string;
  icon: "play.fill" | "doc.text.fill" | "paperplane.fill" | "chart.bar.fill" | "pause.fill" | "stop.fill";
  color: string;
  onPress?: () => void;
}

export function QuickActionCard({ title, icon, color, onPress }: QuickActionCardProps) {
  const colors = useColors();

  return (
    <Pressable
      onPress={onPress}
      style={({ pressed }) => [
        {
          backgroundColor: colors.surface,
          borderRadius: 16,
          padding: 16,
          width: 100,
          alignItems: "center",
          opacity: pressed ? 0.7 : 1,
        },
      ]}
    >
      <View 
        className="w-12 h-12 rounded-full items-center justify-center mb-2"
        style={{ backgroundColor: `${color}20` }}
      >
        <IconSymbol name={icon} size={24} color={color} />
      </View>
      <Text className="text-xs font-medium text-foreground text-center">{title}</Text>
    </Pressable>
  );
}
