import React from 'react';
import { useAuth } from '@/_core/hooks/useAuth';
import { getLoginUrl } from '@/const';
import { Button } from '@/components/ui/button';
import { IconNavigation } from './IconNavigation';
import { DetailSidebar } from './DetailSidebar';
import { WebBrowserPanel } from './WebBrowserPanel';
import { NotificationsPanel } from './NotificationsPanel';
import { FavoritesPanel } from './FavoritesPanel';
import { CommandPalette } from './CommandPalette';
import { Loader2, Maximize2 } from 'lucide-react';
import { useSidebar } from '@/contexts/SidebarContext';
import { motion, AnimatePresence } from 'framer-motion';
import { cn } from '@/lib/utils';

interface TwoLevelSidebarProps
{
  children?: React.ReactNode;
}

// Loading skeleton for the sidebar layout
function SidebarLayoutSkeleton ()
{
  return (
    <div className="flex h-screen w-full bg-[#0A0A0A] items-center justify-center">
      <div className="flex flex-col items-center gap-4">
        <Loader2 className="h-8 w-8 animate-spin text-[#00D4FF]" />
        <p className="text-sm text-[#9BA1A6]">Loading...</p>
      </div>
    </div>
  );
}

// Sign in prompt for unauthenticated users
function SignInPrompt ()
{
  return (
    <div className="flex h-screen w-full bg-[#0A0A0A] items-center justify-center">
      <div className="flex flex-col items-center gap-8 p-8 max-w-md w-full">
        {/* Ripple Logo */ }
        <div className="w-16 h-16 mb-2">
          <img
            src="/images/icon.png"
            alt="Ripple"
            className="w-full h-full object-contain"
          />
        </div>

        <div className="flex flex-col items-center gap-4">
          <h1 className="text-2xl font-semibold tracking-tight text-center text-white">
            Sign in to continue
          </h1>
          <p className="text-sm text-[#9BA1A6] text-center max-w-sm">
            Access to this dashboard requires authentication. Continue to launch the login flow.
          </p>
        </div>

        <Button
          onClick={ () =>
          {
            window.location.href = getLoginUrl();
          } }
          size="lg"
          className="w-full bg-gradient-to-r from-[#00D4FF] to-[#9B7BFF] hover:opacity-90 text-white font-medium shadow-lg hover:shadow-xl transition-all"
        >
          Sign in
        </Button>
      </div>
    </div>
  );
}

// Floating expand button when sidebar is fully collapsed
function FloatingExpandButton ()
{
  const { toggleDoubleMinimize, isFullyCollapsed } = useSidebar();

  if ( !isFullyCollapsed ) return null;

  return (
    <motion.button
      initial={ { opacity: 0, scale: 0.8 } }
      animate={ { opacity: 1, scale: 1 } }
      exit={ { opacity: 0, scale: 0.8 } }
      onClick={ toggleDoubleMinimize }
      className="fixed left-4 top-1/2 -translate-y-1/2 z-50 w-10 h-10 rounded-full bg-[#1A1A1A] border border-[#2A2A2A] flex items-center justify-center shadow-lg hover:bg-[#2A2A2A] transition-colors"
      title="Expand sidebar"
    >
      <Maximize2 size={ 18 } color="#00D4FF" />
    </motion.button>
  );
}

export function TwoLevelSidebar ( { children }: TwoLevelSidebarProps )
{
  const { loading, user } = useAuth();

  // Show loading state
  if ( loading )
  {
    return <SidebarLayoutSkeleton />;
  }

  // Show sign in prompt if not authenticated
  if ( !user )
  {
    return <SignInPrompt />;
  }

  return (
    <TwoLevelSidebarContent>
      { children }
    </TwoLevelSidebarContent>
  );
}

// Inner component that uses sidebar context
function TwoLevelSidebarContent ( { children }: { children?: React.ReactNode; } )
{
  const { iconCollapsed, isFullyCollapsed } = useSidebar();

  return (
    <div className="flex h-screen w-full bg-[#0A0A0A]">
      {/* Sidebar container - animated collapse */ }
      <AnimatePresence>
        { !isFullyCollapsed && (
          <motion.div
            className="flex h-full flex-shrink-0"
            initial={ { width: 'auto', opacity: 1 } }
            animate={ { width: 'auto', opacity: 1 } }
            exit={ { width: 0, opacity: 0 } }
            transition={ { duration: 0.3, ease: 'easeInOut' } }
          >
            <motion.div
              initial={ false }
              animate={ {
                width: iconCollapsed ? 0 : 64,
                opacity: iconCollapsed ? 0 : 1
              } }
              transition={ { duration: 0.3, ease: 'easeInOut' } }
              className="overflow-hidden"
            >
              <IconNavigation />
            </motion.div>
            <DetailSidebar />
          </motion.div>
        ) }
      </AnimatePresence>

      {/* Floating expand button when fully collapsed */ }
      <AnimatePresence>
        <FloatingExpandButton />
      </AnimatePresence>

      {/* Main content area - flex-1 takes remaining space */ }
      { children && (
        <div className="flex-1 h-full overflow-auto bg-[#0F0F0F]">
          { children }
        </div>
      ) }

      {/* Modal Panels */ }
      <NotificationsPanel />
      <FavoritesPanel />
      <CommandPalette />
      <WebBrowserPanel />
    </div>
  );
}

// Standalone sidebar for drawer usage
export function SidebarContent ()
{
  return (
    <div className="flex h-full flex-shrink-0 bg-[#0A0A0A]">
      <IconNavigation />
      <DetailSidebar />
    </div>
  );
}

export default TwoLevelSidebar;
