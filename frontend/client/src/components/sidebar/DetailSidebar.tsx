import React, { useState, useRef, useEffect, useCallback } from 'react';
import { useSidebar } from '@/contexts/SidebarContext';
import { ALL_PANELS, type MenuItem as PanelMenuItem, type MenuSection as PanelMenuSection } from '@/data/sidebar-panels';
import * as LucideIcons from 'lucide-react';
import { cn } from '@/lib/utils';
import { useLocation } from 'wouter';
import { motion, AnimatePresence } from 'framer-motion';

// Ripple brand colors
const COLORS = {
  primary: '#00D4FF',
  secondary: '#9B7BFF',
  accent: '#00E676',
  foreground: '#ECEDEE',
  muted: '#9BA1A6',
  border: '#2A2A2A',
};

// Dynamic icon component
function DynamicIcon({ name, size = 16, color = COLORS.foreground }: { 
  name: string; 
  size?: number; 
  color?: string;
}) {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const IconComponent = (LucideIcons as any)[name] || LucideIcons.Circle;
  return <IconComponent size={size} color={color} />;
}

// Get panel content by ID
function getPanelContent(panelId: string) {
  const panel = ALL_PANELS.find(p => p.id === panelId);
  if (panel) {
    return {
      title: panel.title,
      badge: panel.badge,
      badgeColor: panel.badgeColor,
      sections: panel.sections,
    };
  }
  return {
    title: ALL_PANELS[0].title,
    badge: ALL_PANELS[0].badge,
    badgeColor: ALL_PANELS[0].badgeColor,
    sections: ALL_PANELS[0].sections,
  };
}

interface BadgeProps {
  count?: number;
  color?: string;
  small?: boolean;
}

function Badge({ count, color = '#EF4444', small }: BadgeProps) {
  if (count === undefined) return null;

  return (
    <div 
      className={cn(
        "rounded-full flex items-center justify-center text-white font-semibold",
        small ? "min-w-4 h-4 px-1 text-[9px]" : "min-w-5 h-5 px-1.5 text-[11px]"
      )}
      style={{ backgroundColor: color }}
    >
      {count > 99 ? '99+' : count}
    </div>
  );
}

interface StatusDotProps {
  status?: string;
}

function StatusDot({ status }: StatusDotProps) {
  if (!status) return null;

  const colors: Record<string, string> = {
    online: '#00E676',
    offline: '#666666',
    away: '#FFB300',
    busy: '#FF5252',
  };

  return (
    <div 
      className="absolute -bottom-0.5 -right-0.5 w-1.5 h-1.5 rounded-full border border-black"
      style={{ backgroundColor: colors[status] || '#666666' }}
    />
  );
}

interface MenuItemProps {
  item: PanelMenuItem;
  isFocused: boolean;
  onFocus: () => void;
}

function MenuItem({ item, isFocused, onFocus }: MenuItemProps) {
  const [, setLocation] = useLocation();
  const itemRef = useRef<HTMLButtonElement>(null);

  const handleClick = () => {
    if (item.route) {
      setLocation(item.route);
    }
  };

  // Scroll into view when focused
  useEffect(() => {
    if (isFocused && itemRef.current) {
      itemRef.current.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
    }
  }, [isFocused]);

  return (
    <button
      ref={itemRef}
      onClick={handleClick}
      onMouseEnter={onFocus}
      onFocus={onFocus}
      className={cn(
        "flex items-center justify-between w-full px-3 py-2.5 rounded-lg transition-colors text-left outline-none",
        isFocused 
          ? "bg-[#00D4FF]/10 ring-1 ring-[#00D4FF]/50" 
          : "hover:bg-[rgba(255,255,255,0.05)]"
      )}
    >
      <div className="flex items-center flex-1 gap-3 min-w-0">
        <div className="relative flex-shrink-0">
          <DynamicIcon 
            name={item.icon} 
            size={16} 
            color={isFocused ? COLORS.primary : COLORS.foreground} 
          />
          {item.status && <StatusDot status={item.status} />}
        </div>
        <div className="flex-1 min-w-0">
          <p className={cn(
            "text-sm font-medium truncate",
            isFocused ? "text-[#00D4FF]" : "text-[#ECEDEE]"
          )}>
            {item.label}
          </p>
          {item.metadata && (
            <p className="text-[11px] text-[#9BA1A6] mt-0.5 truncate">
              {item.metadata}
            </p>
          )}
        </div>
      </div>
      <div className="flex items-center gap-2 flex-shrink-0">
        {item.badge !== undefined && <Badge count={item.badge} color={item.badgeColor} small />}
        <LucideIcons.ChevronRight size={16} color={isFocused ? COLORS.primary : COLORS.muted} />
      </div>
    </button>
  );
}

interface MenuSectionProps {
  section: PanelMenuSection;
  focusedItemId: string | null;
  onItemFocus: (itemId: string) => void;
}

function MenuSection({ section, focusedItemId, onItemFocus }: MenuSectionProps) {
  const [isExpanded, setIsExpanded] = useState(section.defaultExpanded !== false);

  return (
    <div className="mb-6">
      <button 
        className="flex items-center justify-between w-full mb-2 px-1"
        onClick={() => section.collapsible && setIsExpanded(!isExpanded)}
      >
        <span className="text-[11px] font-semibold tracking-wider text-[#9BA1A6] uppercase">
          {section.title}
        </span>
        {section.collapsible && (
          <motion.div
            animate={{ rotate: isExpanded ? 0 : -90 }}
            transition={{ duration: 0.2 }}
          >
            <LucideIcons.ChevronDown size={16} color={COLORS.muted} />
          </motion.div>
        )}
      </button>
      <AnimatePresence>
        {isExpanded && (
          <motion.div 
            initial={{ opacity: 0, height: 0 }}
            animate={{ opacity: 1, height: 'auto' }}
            exit={{ opacity: 0, height: 0 }}
            transition={{ duration: 0.2 }}
            className="space-y-0.5 overflow-hidden"
          >
            {section.items.map((item) => (
              <MenuItem 
                key={item.id} 
                item={item} 
                isFocused={focusedItemId === item.id}
                onFocus={() => onItemFocus(item.id)}
              />
            ))}
          </motion.div>
        )}
      </AnimatePresence>
    </div>
  );
}

interface SearchBarProps {
  onOpenCommandPalette?: () => void;
}

function SearchBar({ onOpenCommandPalette }: SearchBarProps) {
  const [searchValue, setSearchValue] = useState('');

  return (
    <div 
      className="flex items-center h-10 rounded-lg border border-[#2A2A2A] px-3 gap-2 mb-5 cursor-pointer hover:border-[#00D4FF]/50 transition-colors"
      onClick={onOpenCommandPalette}
    >
      <LucideIcons.Search size={16} color={COLORS.muted} />
      <input
        type="text"
        className="flex-1 bg-transparent text-sm text-[#ECEDEE] placeholder-[#9BA1A6] outline-none cursor-pointer"
        placeholder="Search or press ⌘K..."
        value={searchValue}
        onChange={(e) => setSearchValue(e.target.value)}
        readOnly
      />
      <div className="px-1.5 py-0.5 rounded bg-[#1A1A1A]">
        <span className="text-[10px] font-medium text-[#9BA1A6]">⌘K</span>
      </div>
    </div>
  );
}

interface HeaderProps {
  title: string;
  badge?: number;
  badgeColor?: string;
  onToggleCollapse: () => void;
  isCollapsed: boolean;
}

function Header({ title, badge, badgeColor, onToggleCollapse, isCollapsed }: HeaderProps) {
  return (
    <div className="flex items-center justify-between mb-4 h-10">
      <div className="flex items-center gap-2">
        <h2 className="text-lg font-semibold text-[#ECEDEE] whitespace-nowrap">{title}</h2>
        {badge !== undefined && <Badge count={badge} color={badgeColor} />}
      </div>
      <button 
        onClick={onToggleCollapse}
        className="w-8 h-8 rounded-lg flex items-center justify-center hover:bg-[rgba(255,255,255,0.05)] transition-colors flex-shrink-0"
      >
        <motion.div
          animate={{ rotate: isCollapsed ? 180 : 0 }}
          transition={{ duration: 0.2 }}
        >
          <LucideIcons.ChevronLeft size={20} color={COLORS.muted} />
        </motion.div>
      </button>
    </div>
  );
}

export function DetailSidebar() {
  const { activeSection, detailCollapsed, setDetailCollapsed, setShowCommandPalette } = useSidebar();
  const [focusedItemId, setFocusedItemId] = useState<string | null>(null);
  const [, setLocation] = useLocation();
  const containerRef = useRef<HTMLDivElement>(null);
  
  const content = getPanelContent(activeSection);

  // Get all items flattened for keyboard navigation
  const getAllItems = useCallback(() => {
    const items: PanelMenuItem[] = [];
    content.sections.forEach(section => {
      items.push(...section.items);
    });
    return items;
  }, [content.sections]);

  // Handle keyboard navigation
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Only handle if sidebar is not collapsed and has focus
      if (detailCollapsed) return;
      
      const items = getAllItems();
      if (items.length === 0) return;

      const currentIndex = focusedItemId 
        ? items.findIndex(item => item.id === focusedItemId)
        : -1;

      switch (e.key) {
        case 'ArrowDown':
        case 'j': // Vim-style navigation
          e.preventDefault();
          const nextIndex = currentIndex < items.length - 1 ? currentIndex + 1 : 0;
          setFocusedItemId(items[nextIndex].id);
          break;
        
        case 'ArrowUp':
        case 'k': // Vim-style navigation
          e.preventDefault();
          const prevIndex = currentIndex > 0 ? currentIndex - 1 : items.length - 1;
          setFocusedItemId(items[prevIndex].id);
          break;
        
        case 'Enter':
          if (focusedItemId) {
            e.preventDefault();
            const focusedItem = items.find(item => item.id === focusedItemId);
            if (focusedItem?.route) {
              setLocation(focusedItem.route);
            }
          }
          break;
        
        case 'Escape':
          setFocusedItemId(null);
          break;
        
        case 'Home':
          e.preventDefault();
          if (items.length > 0) {
            setFocusedItemId(items[0].id);
          }
          break;
        
        case 'End':
          e.preventDefault();
          if (items.length > 0) {
            setFocusedItemId(items[items.length - 1].id);
          }
          break;
      }
    };

    // Only add listener when sidebar container is focused or has focus within
    const container = containerRef.current;
    if (container) {
      container.addEventListener('keydown', handleKeyDown);
      return () => container.removeEventListener('keydown', handleKeyDown);
    }
  }, [detailCollapsed, focusedItemId, getAllItems, setLocation]);

  // Reset focus when section changes
  useEffect(() => {
    setFocusedItemId(null);
  }, [activeSection]);

  return (
    <motion.div 
      ref={containerRef}
      tabIndex={0}
      className="h-full flex-shrink-0 border-r border-[#2A2A2A] overflow-hidden outline-none focus:outline-none"
      style={{ backgroundColor: '#0A0A0A' }}
      initial={false}
      animate={{ 
        width: detailCollapsed ? 0 : 280,
        paddingLeft: detailCollapsed ? 0 : 16,
        paddingRight: detailCollapsed ? 0 : 16,
        paddingTop: detailCollapsed ? 0 : 16,
        paddingBottom: detailCollapsed ? 0 : 16,
      }}
      transition={{ duration: 0.3, ease: 'easeInOut' }}
    >
      <div className="h-full flex flex-col" style={{ width: 248, minWidth: 248 }}>
        <Header
          title={content.title}
          badge={content.badge}
          badgeColor={content.badgeColor}
          onToggleCollapse={() => setDetailCollapsed(!detailCollapsed)}
          isCollapsed={detailCollapsed}
        />
        <SearchBar onOpenCommandPalette={() => setShowCommandPalette(true)} />
        
        {/* Keyboard navigation hint */}
        {focusedItemId && (
          <div className="mb-2 px-2 py-1 bg-[#1A1A1A] rounded text-[10px] text-[#9BA1A6] flex items-center gap-2">
            <span>↑↓ Navigate</span>
            <span>•</span>
            <span>Enter Select</span>
            <span>•</span>
            <span>Esc Clear</span>
          </div>
        )}
        
        <div className="flex-1 overflow-y-auto scrollbar-hide">
          {content.sections.map((section) => (
            <MenuSection 
              key={section.id} 
              section={section}
              focusedItemId={focusedItemId}
              onItemFocus={setFocusedItemId}
            />
          ))}
        </div>
      </div>
    </motion.div>
  );
}

export default DetailSidebar;
