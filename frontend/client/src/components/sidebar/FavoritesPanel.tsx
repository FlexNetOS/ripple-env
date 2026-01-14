import React from 'react';
import { useSidebar } from '@/contexts/SidebarContext';
import * as LucideIcons from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';
import { trpc } from '@/lib/trpc';
import { useLocation } from 'wouter';
import { toast } from 'sonner';

function DynamicIcon({ name, size = 18, color = '#FAFAFA' }: { name: string; size?: number; color?: string }) {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const IconComponent = (LucideIcons as any)[name] || LucideIcons.Circle;
  return <IconComponent size={size} color={color} />;
}

export function FavoritesPanel() {
  const { showFavorites, setShowFavorites, setActiveSection } = useSidebar();
  const [, setLocation] = useLocation();
  const utils = trpc.useUtils();

  // Fetch favorites from database
  const { data: favorites = [], isLoading } = trpc.favorites.list.useQuery(undefined, {
    enabled: showFavorites,
  });

  // Remove favorite mutation
  const removeMutation = trpc.favorites.remove.useMutation({
    onMutate: async ({ itemId }) => {
      // Optimistic update
      await utils.favorites.list.cancel();
      const previousFavorites = utils.favorites.list.getData();
      utils.favorites.list.setData(undefined, (old) => 
        old?.filter(f => f.itemId !== itemId) ?? []
      );
      return { previousFavorites };
    },
    onError: (err, _, context) => {
      utils.favorites.list.setData(undefined, context?.previousFavorites);
      toast.error('Failed to remove favorite');
    },
    onSettled: () => {
      utils.favorites.list.invalidate();
    },
  });

  const removeFavorite = (itemId: string) => {
    removeMutation.mutate({ itemId });
  };

  const navigateToFavorite = (favorite: typeof favorites[0]) => {
    if (favorite.route) {
      setLocation(favorite.route);
    }
    if (favorite.panelId) {
      setActiveSection(favorite.panelId);
    }
    setShowFavorites(false);
  };

  return (
    <AnimatePresence>
      {showFavorites && (
        <>
          {/* Backdrop */}
          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            className="fixed inset-0 bg-black/50 z-50"
            onClick={() => setShowFavorites(false)}
          />
          
          {/* Panel */}
          <motion.div
            initial={{ opacity: 0, x: 20, scale: 0.95 }}
            animate={{ opacity: 1, x: 0, scale: 1 }}
            exit={{ opacity: 0, x: 20, scale: 0.95 }}
            transition={{ duration: 0.2 }}
            className="fixed top-16 right-4 w-[320px] max-h-[70vh] bg-[#171717] rounded-2xl border border-[#262626] overflow-hidden z-50 shadow-2xl"
            onClick={(e) => e.stopPropagation()}
          >
            {/* Header */}
            <div className="flex items-center justify-between p-4 border-b border-[#262626]">
              <div className="flex items-center gap-2">
                <LucideIcons.Star size={20} className="text-yellow-400" />
                <span className="text-base font-semibold text-[#FAFAFA]">Favorites</span>
                {favorites.length > 0 && (
                  <span className="text-xs text-[#6B7280] bg-[#262626] px-2 py-0.5 rounded-full">
                    {favorites.length}
                  </span>
                )}
              </div>
              <button
                onClick={() => setShowFavorites(false)}
                className="p-1.5 rounded-lg hover:bg-[#262626] transition-colors"
              >
                <LucideIcons.X size={18} className="text-[#9CA3AF]" />
              </button>
            </div>

            <p className="text-xs text-[#6B7280] px-4 pt-3 pb-2">
              Quick access to your most used sections
            </p>

            {/* Favorites List */}
            <div className="flex-1 overflow-y-auto p-2 max-h-[calc(70vh-160px)]">
              {isLoading ? (
                <div className="flex flex-col items-center justify-center py-12">
                  <LucideIcons.Loader2 size={32} className="text-[#4B5563] animate-spin" />
                  <span className="text-sm text-[#6B7280] mt-3">Loading favorites...</span>
                </div>
              ) : favorites.length === 0 ? (
                <div className="flex flex-col items-center justify-center py-12">
                  <LucideIcons.StarOff size={48} className="text-[#4B5563]" />
                  <span className="text-sm text-[#6B7280] mt-3">No favorites yet</span>
                  <span className="text-xs text-[#4B5563] mt-1">Star items to add them here</span>
                </div>
              ) : (
                favorites.map((favorite) => (
                  <button
                    key={favorite.id}
                    className="w-full p-3 mb-1 rounded-xl bg-[#1f1f1f] hover:bg-[#262626] text-left transition-colors group"
                    onClick={() => navigateToFavorite(favorite)}
                  >
                    <div className="flex items-center gap-3">
                      <div className="w-8 h-8 rounded-lg bg-gradient-to-br from-[#00D4FF]/20 to-[#9B7BFF]/20 flex items-center justify-center">
                        <DynamicIcon name={favorite.icon || 'Star'} size={16} color="#00D4FF" />
                      </div>
                      <div className="flex-1 min-w-0">
                        <span className="block text-sm font-medium text-[#FAFAFA] truncate">
                          {favorite.title}
                        </span>
                        <span className="block text-xs text-[#6B7280] truncate">
                          {favorite.itemType}
                        </span>
                      </div>
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          removeFavorite(favorite.itemId);
                        }}
                        className="p-1.5 hover:bg-[#333] rounded-lg transition-colors opacity-0 group-hover:opacity-100"
                      >
                        <LucideIcons.X size={14} className="text-[#6B7280]" />
                      </button>
                    </div>
                  </button>
                ))
              )}
            </div>

            {/* Footer */}
            <div className="p-3 border-t border-[#262626]">
              <p className="text-[11px] text-[#4B5563] text-center">
                Right-click any section to add to favorites
              </p>
            </div>
          </motion.div>
        </>
      )}
    </AnimatePresence>
  );
}

export default FavoritesPanel;
