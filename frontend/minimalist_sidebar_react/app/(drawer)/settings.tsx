import React from 'react';
import { ScrollView, Text, View, Pressable, StyleSheet, Switch } from 'react-native';
import { ScreenContainer } from '@/components/screen-container';
import { useColors } from '@/hooks/use-colors';
import { DrawerActions, useNavigation } from '@react-navigation/native';
import MaterialIcons from '@expo/vector-icons/MaterialIcons';

function SettingsSection({ title, children }: { title: string; children: React.ReactNode }) {
  const colors = useColors();
  
  return (
    <View style={styles.section}>
      <Text style={[styles.sectionTitle, { color: colors.muted }]}>{title.toUpperCase()}</Text>
      <View style={[styles.sectionContent, { backgroundColor: colors.surface, borderColor: colors.border }]}>
        {children}
      </View>
    </View>
  );
}

function SettingsRow({ 
  icon, 
  label, 
  value, 
  onPress,
  hasSwitch,
  switchValue,
  onSwitchChange,
}: { 
  icon: keyof typeof MaterialIcons.glyphMap;
  label: string;
  value?: string;
  onPress?: () => void;
  hasSwitch?: boolean;
  switchValue?: boolean;
  onSwitchChange?: (value: boolean) => void;
}) {
  const colors = useColors();

  return (
    <Pressable 
      onPress={onPress}
      style={({ pressed }) => [
        styles.settingsRow,
        { borderBottomColor: colors.border },
        pressed && !hasSwitch && { opacity: 0.7 },
      ]}
    >
      <View style={styles.settingsRowLeft}>
        <MaterialIcons name={icon} size={20} color={colors.muted} />
        <Text style={[styles.settingsLabel, { color: colors.foreground }]}>{label}</Text>
      </View>
      <View style={styles.settingsRowRight}>
        {value && <Text style={[styles.settingsValue, { color: colors.muted }]}>{value}</Text>}
        {hasSwitch && (
          <Switch
            value={switchValue}
            onValueChange={onSwitchChange}
            trackColor={{ false: colors.border, true: '#00D4FF' }}
            thumbColor="#FFFFFF"
          />
        )}
        {!hasSwitch && !value && (
          <MaterialIcons name="chevron-right" size={20} color={colors.muted} />
        )}
      </View>
    </Pressable>
  );
}

export default function SettingsScreen() {
  const colors = useColors();
  const navigation = useNavigation();
  const [notifications, setNotifications] = React.useState(true);
  const [autoScaling, setAutoScaling] = React.useState(true);
  const [darkMode, setDarkMode] = React.useState(true);

  return (
    <ScreenContainer containerClassName="bg-background">
      <ScrollView style={styles.container} showsVerticalScrollIndicator={false}>
        {/* Header */}
        <View style={styles.header}>
          <Pressable
            onPress={() => navigation.dispatch(DrawerActions.toggleDrawer())}
            style={styles.menuButton}
          >
            <MaterialIcons name="menu" size={24} color={colors.foreground} />
          </Pressable>
          <View style={styles.headerContent}>
            <Text style={[styles.title, { color: colors.foreground }]}>Settings</Text>
            <Text style={[styles.subtitle, { color: colors.muted }]}>App Configuration</Text>
          </View>
        </View>

        {/* General */}
        <SettingsSection title="General">
          <SettingsRow 
            icon="dark-mode" 
            label="Dark Mode" 
            hasSwitch 
            switchValue={darkMode}
            onSwitchChange={setDarkMode}
          />
          <SettingsRow 
            icon="notifications" 
            label="Notifications" 
            hasSwitch 
            switchValue={notifications}
            onSwitchChange={setNotifications}
          />
          <SettingsRow icon="language" label="Language" value="English" />
        </SettingsSection>

        {/* System */}
        <SettingsSection title="System">
          <SettingsRow 
            icon="auto-graph" 
            label="Auto-Scaling" 
            hasSwitch 
            switchValue={autoScaling}
            onSwitchChange={setAutoScaling}
          />
          <SettingsRow icon="terminal" label="Environment Variables" />
          <SettingsRow icon="vpn-key" label="API Keys" />
          <SettingsRow icon="extension" label="Integrations" />
        </SettingsSection>

        {/* Build */}
        <SettingsSection title="Build Configuration">
          <SettingsRow icon="folder" label="Build Output Path" value="/dist" />
          <SettingsRow icon="memory" label="Max Memory" value="16 GB" />
          <SettingsRow icon="schedule" label="Build Timeout" value="30 min" />
        </SettingsSection>

        {/* Network */}
        <SettingsSection title="Network">
          <SettingsRow icon="public" label="API Endpoint" value="localhost:3000" />
          <SettingsRow icon="hub" label="Holochain Network" value="Connected" />
          <SettingsRow icon="security" label="TLS/SSL" value="Enabled" />
        </SettingsSection>

        {/* About */}
        <SettingsSection title="About">
          <SettingsRow icon="info" label="Version" value="1.0.0" />
          <SettingsRow icon="menu-book" label="Documentation" />
          <SettingsRow icon="help" label="Support" />
          <SettingsRow icon="code" label="Open Source Licenses" />
        </SettingsSection>

        {/* Danger Zone */}
        <View style={styles.dangerSection}>
          <Text style={[styles.dangerTitle, { color: '#FF5252' }]}>DANGER ZONE</Text>
          <View style={[styles.dangerContent, { borderColor: '#FF5252' }]}>
            <Pressable style={[styles.dangerButton, { backgroundColor: 'rgba(255, 82, 82, 0.1)' }]}>
              <MaterialIcons name="delete-forever" size={20} color="#FF5252" />
              <Text style={[styles.dangerButtonText, { color: '#FF5252' }]}>Reset All Settings</Text>
            </Pressable>
            <Pressable style={[styles.dangerButton, { backgroundColor: 'rgba(255, 82, 82, 0.1)' }]}>
              <MaterialIcons name="cleaning-services" size={20} color="#FF5252" />
              <Text style={[styles.dangerButtonText, { color: '#FF5252' }]}>Clear Build Cache</Text>
            </Pressable>
          </View>
        </View>
      </ScrollView>
    </ScreenContainer>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    padding: 16,
  },
  header: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 24,
  },
  menuButton: {
    width: 40,
    height: 40,
    borderRadius: 8,
    alignItems: 'center',
    justifyContent: 'center',
    marginRight: 12,
  },
  headerContent: {
    flex: 1,
  },
  title: {
    fontSize: 28,
    fontWeight: '700',
  },
  subtitle: {
    fontSize: 14,
    marginTop: 2,
  },
  section: {
    marginBottom: 24,
  },
  sectionTitle: {
    fontSize: 11,
    fontWeight: '600',
    letterSpacing: 0.5,
    marginBottom: 8,
    marginLeft: 4,
  },
  sectionContent: {
    borderRadius: 12,
    borderWidth: 1,
    overflow: 'hidden',
  },
  settingsRow: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: 16,
    borderBottomWidth: 1,
  },
  settingsRowLeft: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
  },
  settingsRowRight: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  settingsLabel: {
    fontSize: 15,
  },
  settingsValue: {
    fontSize: 14,
  },
  dangerSection: {
    marginBottom: 32,
  },
  dangerTitle: {
    fontSize: 11,
    fontWeight: '600',
    letterSpacing: 0.5,
    marginBottom: 8,
    marginLeft: 4,
  },
  dangerContent: {
    borderRadius: 12,
    borderWidth: 1,
    overflow: 'hidden',
    gap: 8,
    padding: 12,
  },
  dangerButton: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'center',
    gap: 8,
    padding: 14,
    borderRadius: 8,
  },
  dangerButtonText: {
    fontSize: 14,
    fontWeight: '600',
  },
});
