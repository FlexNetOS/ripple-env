// Fallback for using MaterialIcons on Android and web.

import MaterialIcons from "@expo/vector-icons/MaterialIcons";
import { SymbolWeight, SymbolViewProps } from "expo-symbols";
import { ComponentProps } from "react";
import { OpaqueColorValue, type StyleProp, type TextStyle } from "react-native";

type IconMapping = Record<SymbolViewProps["name"], ComponentProps<typeof MaterialIcons>["name"]>;
type IconSymbolName = keyof typeof MAPPING;

/**
 * Add your SF Symbols to Material Icons mappings here.
 * - see Material Icons in the [Icons Directory](https://icons.expo.fyi).
 * - see SF Symbols in the [SF Symbols](https://developer.apple.com/sf-symbols/) app.
 */
const MAPPING = {
  // Navigation
  "house.fill": "home",
  "paperplane.fill": "send",
  "chevron.left.forwardslash.chevron.right": "code",
  "chevron.right": "chevron-right",
  
  // Ripple app icons
  "hammer.fill": "build",
  "cpu.fill": "memory",
  "person.2.fill": "people",
  "server.rack": "dns",
  "gearshape.fill": "settings",
  "chart.bar.fill": "bar-chart",
  "bolt.fill": "flash-on",
  "checkmark.circle.fill": "check-circle",
  "exclamationmark.triangle.fill": "warning",
  "xmark.circle.fill": "cancel",
  "clock.fill": "schedule",
  "arrow.clockwise": "refresh",
  "play.fill": "play-arrow",
  "pause.fill": "pause",
  "stop.fill": "stop",
  "doc.text.fill": "description",
  "folder.fill": "folder",
  "bell.fill": "notifications",
  "magnifyingglass": "search",
  "plus": "add",
  "minus": "remove",
  "ellipsis": "more-horiz",
  "info.circle.fill": "info",
  "network": "hub",
  "cube.fill": "view-in-ar",
  "terminal.fill": "terminal",
  "shield.fill": "security",
  "lock.fill": "lock",
  "wifi": "wifi",
  "antenna.radiowaves.left.and.right": "cell-tower",
} as IconMapping;

/**
 * An icon component that uses native SF Symbols on iOS, and Material Icons on Android and web.
 * This ensures a consistent look across platforms, and optimal resource usage.
 * Icon `name`s are based on SF Symbols and require manual mapping to Material Icons.
 */
export function IconSymbol({
  name,
  size = 24,
  color,
  style,
}: {
  name: IconSymbolName;
  size?: number;
  color: string | OpaqueColorValue;
  style?: StyleProp<TextStyle>;
  weight?: SymbolWeight;
}) {
  return <MaterialIcons color={color} size={size} name={MAPPING[name]} style={style} />;
}
