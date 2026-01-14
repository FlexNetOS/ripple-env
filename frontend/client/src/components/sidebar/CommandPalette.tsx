import React, { useState, useEffect, useRef, useMemo } from 'react';
import { useSidebar } from '@/contexts/SidebarContext';
import { ALL_PANELS, SECTION_ROUTES } from '@/data/sidebar-panels';
import * as LucideIcons from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';
import { useLocation } from 'wouter';

interface CommandItem {
  id: string;
  label: string;
  description?: string;
  icon: string;
  category: 'navigation' | 'action' | 'recent';
  action: () => void;
}

function DynamicIcon({ name, size = 16, color = '#9CA3AF' }: { name: string; size?: number; color?: string }) {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const IconComponent = (LucideIcons as any)[name] || LucideIcons.Circle;
  return <IconComponent size={size} color={color} />;
}

export function CommandPalette() {
  const { showCommandPalette, setShowCommandPalette, setActiveSection } = useSidebar();
  const [, setLocation] = useLocation();
  const [search, setSearch] = useState('');
  const [selectedIndex, setSelectedIndex] = useState(0);
  const inputRef = useRef<HTMLInputElement>(null);

  // Build command list from panels and routes
  const commands = useMemo<CommandItem[]>(() => {
    const navCommands: CommandItem[] = ALL_PANELS.map(panel => ({
      id: `nav-${panel.id}`,
      label: panel.title,
      description: `Go to ${panel.title}`,
      icon: panel.icon,
      category: 'navigation',
      action: () => {
        setActiveSection(panel.id);
        const route = SECTION_ROUTES[panel.id];
        if (route) {
          setLocation(route);
        }
        setShowCommandPalette(false);
      },
    }));

    const actionCommands: CommandItem[] = [
      {
        id: 'action-notifications',
        label: 'View Notifications',
        description: 'Open notifications panel',
        icon: 'Bell',
        category: 'action',
        action: () => {
          setShowCommandPalette(false);
        },
      },
      {
        id: 'action-favorites',
        label: 'View Favorites',
        description: 'Open favorites panel',
        icon: 'Star',
        category: 'action',
        action: () => {
          setShowCommandPalette(false);
        },
      },
      {
        id: 'action-settings',
        label: 'Settings',
        description: 'Open settings page',
        icon: 'Settings',
        category: 'action',
        action: () => {
          setLocation('/settings');
          setShowCommandPalette(false);
        },
      },
      {
        id: 'action-chat',
        label: 'New Chat',
        description: 'Start a new AI conversation',
        icon: 'MessageSquare',
        category: 'action',
        action: () => {
          setLocation('/chat');
          setShowCommandPalette(false);
        },
      },
      {
        id: 'action-files',
        label: 'Upload File',
        description: 'Upload a new file',
        icon: 'Upload',
        category: 'action',
        action: () => {
          setLocation('/files');
          setShowCommandPalette(false);
        },
      },
    ];

    return [...navCommands, ...actionCommands];
  }, [setActiveSection, setLocation, setShowCommandPalette]);

  // Filter commands based on search
  const filteredCommands = useMemo(() => {
    if (!search.trim()) return commands;
    const searchLower = search.toLowerCase();
    return commands.filter(cmd => 
      cmd.label.toLowerCase().includes(searchLower) ||
      cmd.description?.toLowerCase().includes(searchLower)
    );
  }, [commands, search]);

  // Reset selection when search changes
  useEffect(() => {
    setSelectedIndex(0);
  }, [search]);

  // Focus input when opened
  useEffect(() => {
    if (showCommandPalette && inputRef.current) {
      inputRef.current.focus();
      setSearch('');
      setSelectedIndex(0);
    }
  }, [showCommandPalette]);

  // Keyboard navigation
  useEffect(() => {
    if (!showCommandPalette) return;

    const handleKeyDown = (e: KeyboardEvent) => {
      switch (e.key) {
        case 'ArrowDown':
          e.preventDefault();
          setSelectedIndex(prev => 
            prev < filteredCommands.length - 1 ? prev + 1 : 0
          );
          break;
        case 'ArrowUp':
          e.preventDefault();
          setSelectedIndex(prev => 
            prev > 0 ? prev - 1 : filteredCommands.length - 1
          );
          break;
        case 'Enter':
          e.preventDefault();
          if (filteredCommands[selectedIndex]) {
            filteredCommands[selectedIndex].action();
          }
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [showCommandPalette, filteredCommands, selectedIndex]);

  // Group commands by category
  const groupedCommands = useMemo(() => {
    const groups: Record<string, CommandItem[]> = {
      navigation: [],
      action: [],
      recent: [],
    };
    filteredCommands.forEach(cmd => {
      groups[cmd.category].push(cmd);
    });
    return groups;
  }, [filteredCommands]);

  let currentIndex = 0;

  return (
    <AnimatePresence>
      {showCommandPalette && (
        <>
          {/* Backdrop */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            className="fixed inset-0 bg-black/60 z-50 backdrop-blur-sm"
            onClick={() => setShowCommandPalette(false)}
          />
          
          {/* Command Palette */}
          <motion.div
            initial={{ opacity: 0, y: -20, scale: 0.95 }}
            animate={{ opacity: 1, y: 0, scale: 1 }}
            exit={{ opacity: 0, y: -20, scale: 0.95 }}
            transition={{ duration: 0.15 }}
            className="fixed top-[20%] left-1/2 -translate-x-1/2 w-[560px] max-h-[60vh] bg-[#171717] rounded-2xl border border-[#262626] overflow-hidden z-50 shadow-2xl"
            onClick={(e) => e.stopPropagation()}
          >
            {/* Search Input */}
            <div className="flex items-center gap-3 p-4 border-b border-[#262626]">
              <LucideIcons.Search size={20} className="text-[#6B7280] flex-shrink-0" />
              <input
                ref={inputRef}
                type="text"
                value={search}
                onChange={(e) => setSearch(e.target.value)}
                placeholder="Search commands..."
                className="flex-1 bg-transparent text-[#FAFAFA] text-base placeholder-[#6B7280] outline-none"
              />
              <div className="flex items-center gap-1">
                <kbd className="px-2 py-1 text-xs font-medium text-[#6B7280] bg-[#262626] rounded">
                  esc
                </kbd>
              </div>
            </div>

            {/* Results */}
            <div className="overflow-y-auto max-h-[calc(60vh-80px)] p-2">
              {filteredCommands.length === 0 ? (
                <div className="flex flex-col items-center justify-center py-12">
                  <LucideIcons.SearchX size={40} className="text-[#4B5563]" />
                  <span className="text-sm text-[#6B7280] mt-3">No results found</span>
                  <span className="text-xs text-[#4B5563] mt-1">Try a different search term</span>
                </div>
              ) : (
                <>
                  {/* Navigation Commands */}
                  {groupedCommands.navigation.length > 0 && (
                    <div className="mb-4">
                      <p className="text-[11px] font-semibold text-[#6B7280] uppercase tracking-wider px-3 mb-2">
                        Navigation
                      </p>
                      {groupedCommands.navigation.map((cmd) => {
                        const index = currentIndex++;
                        return (
                          <button
                            key={cmd.id}
                            className={`w-full flex items-center gap-3 px-3 py-2.5 rounded-lg text-left transition-colors ${
                              index === selectedIndex 
                                ? 'bg-[#00D4FF]/10 text-[#00D4FF]' 
                                : 'hover:bg-[#262626] text-[#FAFAFA]'
                            }`}
                            onClick={cmd.action}
                            onMouseEnter={() => setSelectedIndex(index)}
                          >
                            <DynamicIcon 
                              name={cmd.icon} 
                              size={18} 
                              color={index === selectedIndex ? '#00D4FF' : '#9CA3AF'} 
                            />
                            <div className="flex-1 min-w-0">
                              <p className="text-sm font-medium truncate">{cmd.label}</p>
                              {cmd.description && (
                                <p className="text-xs text-[#6B7280] truncate">{cmd.description}</p>
                              )}
                            </div>
                            {index === selectedIndex && (
                              <kbd className="px-2 py-0.5 text-[10px] font-medium text-[#6B7280] bg-[#262626] rounded">
                                ↵
                              </kbd>
                            )}
                          </button>
                        );
                      })}
                    </div>
                  )}

                  {/* Action Commands */}
                  {groupedCommands.action.length > 0 && (
                    <div>
                      <p className="text-[11px] font-semibold text-[#6B7280] uppercase tracking-wider px-3 mb-2">
                        Actions
                      </p>
                      {groupedCommands.action.map((cmd) => {
                        const index = currentIndex++;
                        return (
                          <button
                            key={cmd.id}
                            className={`w-full flex items-center gap-3 px-3 py-2.5 rounded-lg text-left transition-colors ${
                              index === selectedIndex 
                                ? 'bg-[#00D4FF]/10 text-[#00D4FF]' 
                                : 'hover:bg-[#262626] text-[#FAFAFA]'
                            }`}
                            onClick={cmd.action}
                            onMouseEnter={() => setSelectedIndex(index)}
                          >
                            <DynamicIcon 
                              name={cmd.icon} 
                              size={18} 
                              color={index === selectedIndex ? '#00D4FF' : '#9CA3AF'} 
                            />
                            <div className="flex-1 min-w-0">
                              <p className="text-sm font-medium truncate">{cmd.label}</p>
                              {cmd.description && (
                                <p className="text-xs text-[#6B7280] truncate">{cmd.description}</p>
                              )}
                            </div>
                            {index === selectedIndex && (
                              <kbd className="px-2 py-0.5 text-[10px] font-medium text-[#6B7280] bg-[#262626] rounded">
                                ↵
                              </kbd>
                            )}
                          </button>
                        );
                      })}
                    </div>
                  )}
                </>
              )}
            </div>

            {/* Footer */}
            <div className="flex items-center justify-between px-4 py-2 border-t border-[#262626] bg-[#0f0f0f]">
              <div className="flex items-center gap-4 text-[11px] text-[#6B7280]">
                <span className="flex items-center gap-1">
                  <kbd className="px-1.5 py-0.5 bg-[#262626] rounded text-[10px]">↑</kbd>
                  <kbd className="px-1.5 py-0.5 bg-[#262626] rounded text-[10px]">↓</kbd>
                  navigate
                </span>
                <span className="flex items-center gap-1">
                  <kbd className="px-1.5 py-0.5 bg-[#262626] rounded text-[10px]">↵</kbd>
                  select
                </span>
              </div>
              <span className="text-[11px] text-[#6B7280]">
                {filteredCommands.length} results
              </span>
            </div>
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}

export default CommandPalette;
