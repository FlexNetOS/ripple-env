import { ScrollView, Text, View, Pressable, Switch } from "react-native";
import { useState } from "react";
import { ScreenContainer } from "@/components/screen-container";
import { useColors } from "@/hooks/use-colors";
import { IconSymbol } from "@/components/ui/icon-symbol";

interface SettingRowProps {
  icon: "bell.fill" | "shield.fill" | "wifi" | "folder.fill" | "info.circle.fill" | "doc.text.fill";
  title: string;
  subtitle?: string;
  rightElement?: React.ReactNode;
  onPress?: () => void;
}

function SettingRow({ icon, title, subtitle, rightElement, onPress }: SettingRowProps) {
  const colors = useColors();

  return (
    <Pressable
      onPress={onPress}
      style={({ pressed }) => [
        {
          flexDirection: "row",
          alignItems: "center",
          paddingVertical: 14,
          paddingHorizontal: 16,
          borderBottomWidth: 1,
          borderBottomColor: colors.border,
          opacity: pressed && onPress ? 0.7 : 1,
        },
      ]}
    >
      <View 
        className="w-9 h-9 rounded-lg items-center justify-center mr-3"
        style={{ backgroundColor: `${colors.primary}15` }}
      >
        <IconSymbol name={icon} size={18} color={colors.primary} />
      </View>
      <View className="flex-1">
        <Text className="text-base text-foreground">{title}</Text>
        {subtitle && <Text className="text-xs text-muted mt-0.5">{subtitle}</Text>}
      </View>
      {rightElement || (
        <IconSymbol name="chevron.right" size={18} color={colors.muted} />
      )}
    </Pressable>
  );
}

function SettingSection({ title, children }: { title: string; children: React.ReactNode }) {
  const colors = useColors();

  return (
    <View className="mb-6">
      <Text className="text-xs font-semibold text-muted uppercase tracking-wider px-5 mb-2">
        {title}
      </Text>
      <View 
        className="mx-5 rounded-xl overflow-hidden"
        style={{ backgroundColor: colors.surface }}
      >
        {children}
      </View>
    </View>
  );
}

export default function SettingsScreen() {
  const colors = useColors();
  const [notifications, setNotifications] = useState(true);
  const [darkMode, setDarkMode] = useState(true);
  const [autoSync, setAutoSync] = useState(true);

  return (
    <ScreenContainer>
      <ScrollView 
        className="flex-1"
        contentContainerStyle={{ paddingBottom: 24 }}
        showsVerticalScrollIndicator={false}
      >
        {/* Header */}
        <View className="px-5 pt-4 pb-6">
          <Text className="text-2xl font-bold text-foreground">Settings</Text>
          <Text className="text-sm text-muted mt-1">Configure your Ripple instance</Text>
        </View>

        {/* Profile Section */}
        <View className="px-5 mb-6">
          <View 
            className="p-4 rounded-xl flex-row items-center"
            style={{ backgroundColor: colors.surface }}
          >
            <View 
              className="w-14 h-14 rounded-full items-center justify-center mr-4"
              style={{ backgroundColor: `${colors.primary}20` }}
            >
              <Text className="text-xl font-bold" style={{ color: colors.primary }}>R</Text>
            </View>
            <View className="flex-1">
              <Text className="text-lg font-semibold text-foreground">Ripple Admin</Text>
              <Text className="text-sm text-muted">admin@ripple.local</Text>
            </View>
            <IconSymbol name="chevron.right" size={20} color={colors.muted} />
          </View>
        </View>

        {/* Preferences */}
        <SettingSection title="Preferences">
          <SettingRow
            icon="bell.fill"
            title="Notifications"
            subtitle="Push alerts for build events"
            rightElement={
              <Switch
                value={notifications}
                onValueChange={setNotifications}
                trackColor={{ false: colors.border, true: colors.primary }}
                thumbColor={colors.background}
              />
            }
          />
          <SettingRow
            icon="wifi"
            title="Auto Sync"
            subtitle="Sync status in real-time"
            rightElement={
              <Switch
                value={autoSync}
                onValueChange={setAutoSync}
                trackColor={{ false: colors.border, true: colors.primary }}
                thumbColor={colors.background}
              />
            }
          />
        </SettingSection>

        {/* Build Configuration */}
        <SettingSection title="Build Configuration">
          <SettingRow
            icon="folder.fill"
            title="Workspace Path"
            subtitle="/home/ubuntu/ripple_workspace"
            onPress={() => {}}
          />
          <SettingRow
            icon="doc.text.fill"
            title="Vendor Lock"
            subtitle="69 archives, all verified"
            onPress={() => {}}
          />
          <SettingRow
            icon="shield.fill"
            title="Security"
            subtitle="Cosign signatures enabled"
            onPress={() => {}}
          />
        </SettingSection>

        {/* System Info */}
        <SettingSection title="System">
          <SettingRow
            icon="info.circle.fill"
            title="Version"
            subtitle="Ripple BuildKit v1.0.0"
            rightElement={null}
          />
          <SettingRow
            icon="doc.text.fill"
            title="Documentation"
            onPress={() => {}}
          />
          <SettingRow
            icon="info.circle.fill"
            title="About Ripple"
            onPress={() => {}}
          />
        </SettingSection>

        {/* Build Info Card */}
        <View className="px-5">
          <View 
            className="p-4 rounded-xl"
            style={{ backgroundColor: `${colors.primary}10` }}
          >
            <Text className="text-sm font-semibold mb-2" style={{ color: colors.primary }}>
              Ripple BuildKit
            </Text>
            <Text className="text-xs text-muted leading-5">
              AI-driven build orchestration system for Holochain-native agentic operating systems. 
              Produces statically-linked bootstrap binaries for Windows, Linux, and macOS.
            </Text>
            <View className="flex-row flex-wrap gap-2 mt-3">
              <View 
                className="px-2 py-1 rounded"
                style={{ backgroundColor: colors.background }}
              >
                <Text className="text-xs text-muted">NixOS</Text>
              </View>
              <View 
                className="px-2 py-1 rounded"
                style={{ backgroundColor: colors.background }}
              >
                <Text className="text-xs text-muted">Holochain</Text>
              </View>
              <View 
                className="px-2 py-1 rounded"
                style={{ backgroundColor: colors.background }}
              >
                <Text className="text-xs text-muted">Rust</Text>
              </View>
              <View 
                className="px-2 py-1 rounded"
                style={{ backgroundColor: colors.background }}
              >
                <Text className="text-xs text-muted">Nushell</Text>
              </View>
            </View>
          </View>
        </View>
      </ScrollView>
    </ScreenContainer>
  );
}
