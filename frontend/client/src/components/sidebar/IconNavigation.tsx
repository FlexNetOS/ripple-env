import React, { useState } from 'react';
import { useSidebar } from '@/contexts/SidebarContext';
import { 
  WORKSPACES, 
  ALL_PANELS, 
  getPanelsForWorkspace, 
  type WorkspaceType,
  type SidebarPanel,
} from '@/data/sidebar-panels';
import * as LucideIcons from 'lucide-react';
import { cn } from '@/lib/utils';

// Ripple brand colors
const COLORS = {
  primary: '#00D4FF',
  secondary: '#9B7BFF',
  accent: '#00E676',
  muted: '#9BA1A6',
  border: '#2A2A2A',
};

// Dynamic icon component
function DynamicIcon({ name, size = 20, color = COLORS.muted }: { 
  name: string; 
  size?: number; 
  color?: string;
}) {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const IconComponent = (LucideIcons as any)[name] || LucideIcons.Circle;
  return <IconComponent size={size} color={color} />;
}

interface BadgeProps {
  count?: number;
  color?: string;
}

function Badge({ count, color = '#EF4444' }: BadgeProps) {
  if (count === undefined) return null;

  return (
    <div 
      className="absolute -top-1 -right-1 min-w-[18px] h-[18px] rounded-full flex items-center justify-center px-1 border-2 border-black text-[10px] font-semibold text-white"
      style={{ backgroundColor: color }}
    >
      {count > 99 ? '99+' : count}
    </div>
  );
}

interface IconNavButtonProps {
  panel: SidebarPanel;
  isActive: boolean;
  onPress: () => void;
}

function IconNavButton({ panel, isActive, onPress }: IconNavButtonProps) {
  return (
    <button 
      onClick={onPress}
      className={cn(
        "w-10 h-10 rounded-[10px] flex items-center justify-center relative transition-all duration-200",
        isActive ? "bg-[rgba(0,212,255,0.15)]" : "hover:bg-[rgba(255,255,255,0.05)]"
      )}
    >
      <DynamicIcon 
        name={panel.icon} 
        size={20} 
        color={isActive ? COLORS.primary : COLORS.muted} 
      />
      {panel.badge !== undefined && (
        <Badge count={panel.badge} color={panel.badgeColor} />
      )}
    </button>
  );
}

// Ripple Logo Component
function RippleLogo({ size = 32 }: { size?: number }) {
  return (
    <div 
      className="flex items-center justify-center"
      style={{ width: size, height: size }}
    >
      <img 
        src="/images/icon.png" 
        alt="Ripple" 
        className="w-full h-full object-contain"
        onError={(e) => {
          // Fallback to SVG if image fails to load
          e.currentTarget.style.display = 'none';
          e.currentTarget.nextElementSibling?.classList.remove('hidden');
        }}
      />
      {/* Fallback SVG */}
      <svg 
        className="hidden"
        width={size} 
        height={size} 
        viewBox="0 0 32 32" 
        fill="none"
      >
        <circle cx="16" cy="16" r="14" stroke="url(#ripple-gradient)" strokeWidth="2" />
        <circle cx="16" cy="16" r="8" fill="url(#ripple-gradient)" />
        <defs>
          <linearGradient id="ripple-gradient" x1="0" y1="0" x2="32" y2="32">
            <stop offset="0%" stopColor="#00D4FF" />
            <stop offset="50%" stopColor="#9B7BFF" />
            <stop offset="100%" stopColor="#00E676" />
          </linearGradient>
        </defs>
      </svg>
    </div>
  );
}

export function IconNavigation() {
  const { 
    activeSection, 
    setActiveSection, 
    activeWorkspace, 
    setActiveWorkspace,
    detailCollapsed,
    setDetailCollapsed,
    setShowNotifications,
    setShowFavorites,
  } = useSidebar();
  
  const [showWorkspaceMenu, setShowWorkspaceMenu] = useState(false);

  // Get panels for current workspace
  const panels = getPanelsForWorkspace(activeWorkspace);
  
  // Separate main panels from bottom panels
  const bottomPanelIds = ['notifications', 'favorites', 'settings', 'profile'];
  const mainPanels = panels.filter(p => !bottomPanelIds.includes(p.id));
  const bottomPanels = ALL_PANELS.filter(p => bottomPanelIds.includes(p.id));

  const handleWorkspaceSelect = (wsId: WorkspaceType) => {
    setActiveWorkspace(wsId);
    setShowWorkspaceMenu(false);
    // Select first panel of new workspace
    const newPanels = getPanelsForWorkspace(wsId);
    if (newPanels.length > 0 && !bottomPanelIds.includes(newPanels[0].id)) {
      setActiveSection(newPanels[0].id);
    }
  };

  const handleBottomPanelClick = (panelId: string) => {
    if (panelId === 'notifications') {
      setShowNotifications(true);
    } else if (panelId === 'favorites') {
      setShowFavorites(true);
    } else {
      setActiveSection(panelId);
    }
  };

  return (
    <div 
      className="w-16 h-full flex flex-col items-center py-4 border-r"
      style={{ backgroundColor: '#000000', borderRightColor: COLORS.border }}
    >
      {/* Ripple Logo */}
      <div className="mb-4">
        <RippleLogo size={36} />
      </div>

      {/* Workspace Switcher */}
      <div className="relative mb-3">
        <button
          onClick={() => setShowWorkspaceMenu(!showWorkspaceMenu)}
          className="w-10 h-10 rounded-xl flex items-center justify-center bg-[#1F2937] hover:opacity-80 transition-opacity"
        >
          <LucideIcons.Layers size={20} color={COLORS.primary} />
        </button>
        {/* Badge */}
        <div className="absolute -top-1 -right-1 bg-[#22C55E] rounded-lg min-w-4 h-4 flex items-center justify-center px-1 border-2 border-black">
          <span className="text-white text-[10px] font-semibold">3</span>
        </div>

        {/* Workspace Dropdown */}
        {showWorkspaceMenu && (
          <>
            <div 
              className="fixed inset-0 z-40"
              onClick={() => setShowWorkspaceMenu(false)}
            />
            <div className="absolute left-full top-0 ml-2 z-50 bg-[#1F2937] rounded-lg p-2 w-40 shadow-lg">
              <p className="text-[#6B7280] text-[11px] font-semibold tracking-wider px-2 py-1 mb-1">
                WORKSPACE
              </p>
              {WORKSPACES.map((ws) => (
                <button
                  key={ws.id}
                  className={cn(
                    "flex items-center gap-2 w-full px-2 py-2 rounded-md text-left transition-colors",
                    activeWorkspace === ws.id 
                      ? "bg-[rgba(0,217,255,0.1)] text-white" 
                      : "text-[#9CA3AF] hover:bg-[rgba(255,255,255,0.05)]"
                  )}
                  onClick={() => handleWorkspaceSelect(ws.id as WorkspaceType)}
                >
                  <DynamicIcon 
                    name={ws.icon} 
                    size={16} 
                    color={activeWorkspace === ws.id ? COLORS.primary : '#9CA3AF'} 
                  />
                  <span className="text-sm">{ws.label}</span>
                </button>
              ))}
            </div>
          </>
        )}
      </div>

      {/* Collapse/Expand Toggle */}
      <button
        onClick={() => setDetailCollapsed(!detailCollapsed)}
        className={cn(
          "w-10 h-10 rounded-[10px] flex items-center justify-center mb-2 transition-all duration-200",
          "hover:bg-[rgba(255,255,255,0.05)]"
        )}
        title={detailCollapsed ? "Expand sidebar" : "Collapse sidebar"}
      >
        {detailCollapsed ? (
          <LucideIcons.PanelLeftOpen size={18} color={COLORS.muted} />
        ) : (
          <LucideIcons.PanelLeftClose size={18} color={COLORS.muted} />
        )}
      </button>

      {/* Navigation Icons - Scrollable */}
      <div className="flex-1 w-full overflow-y-auto scrollbar-hide">
        <div className="flex flex-col items-center gap-1.5 py-1">
          {mainPanels.map((panel) => (
            <IconNavButton
              key={panel.id}
              panel={panel}
              isActive={activeSection === panel.id}
              onPress={() => setActiveSection(panel.id)}
            />
          ))}
        </div>
      </div>

      {/* Bottom Icons */}
      <div className="flex flex-col items-center gap-2 pt-3 border-t border-[#262626]">
        {bottomPanels.slice(0, 3).map((panel) => (
          <IconNavButton
            key={panel.id}
            panel={panel}
            isActive={activeSection === panel.id}
            onPress={() => handleBottomPanelClick(panel.id)}
          />
        ))}
        
        {/* Avatar/Profile */}
        <button
          className={cn(
            "w-8 h-8 rounded-full bg-[#171717] flex items-center justify-center border mt-1 transition-colors",
            activeSection === 'profile' ? "border-[#00D4FF]" : "border-[#262626] hover:border-[#404040]"
          )}
          onClick={() => setActiveSection('profile')}
        >
          <LucideIcons.User size={16} color="#FAFAFA" />
        </button>
      </div>
    </div>
  );
}

export default IconNavigation;
